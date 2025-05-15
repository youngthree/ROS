import os
import asyncio
import json
from aiohttp import web
import aiohttp
import cv2
import numpy as np

# Environment Variables
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
DEVICE_ROSBRIDGE_PORT = int(os.environ.get("DEVICE_ROSBRIDGE_PORT", "9090"))
DEVICE_RTSP_PORT = int(os.environ.get("DEVICE_RTSP_PORT", "554"))
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", "8080"))
RTSP_PATH = os.environ.get("RTSP_PATH", "/stream")  # Path after rtsp://IP:PORT

# For ROSBridge websocket communication
ROSBRIDGE_URI = f"ws://{DEVICE_IP}:{DEVICE_ROSBRIDGE_PORT}"

# For RTSP video streaming
RTSP_URL = f"rtsp://{DEVICE_IP}:{DEVICE_RTSP_PORT}{RTSP_PATH}"

# --- /status helper: Fetch localization/navigation state from ROSBridge ---
async def fetch_status():
    # Using aiohttp websocket to ROSBridge
    async with aiohttp.ClientSession() as session:
        async with session.ws_connect(ROSBRIDGE_URI) as ws:
            # Subscribe to relevant status topics (e.g., /odom, /imu, /navigation_status)
            subscribe_msgs = [
                {
                    "op": "subscribe",
                    "topic": "/odom",
                    "type": "nav_msgs/Odometry",
                    "id": "odom_sub"
                },
                {
                    "op": "subscribe",
                    "topic": "/imu",
                    "type": "sensor_msgs/Imu",
                    "id": "imu_sub"
                },
                {
                    "op": "subscribe",
                    "topic": "/navigation_status",
                    "id": "nav_status_sub"
                }
            ]
            for msg in subscribe_msgs:
                await ws.send_json(msg)
            status = {}
            required = {"odom": False, "imu": False, "navigation_status": False}
            async for msg in ws:
                if msg.type == aiohttp.WSMsgType.TEXT:
                    data = json.loads(msg.data)
                    if data.get("topic") == "/odom" and not required["odom"]:
                        status["odom"] = data.get("msg", {})
                        required["odom"] = True
                    elif data.get("topic") == "/imu" and not required["imu"]:
                        status["imu"] = data.get("msg", {})
                        required["imu"] = True
                    elif data.get("topic") == "/navigation_status" and not required["navigation_status"]:
                        status["navigation_status"] = data.get("msg", {})
                        required["navigation_status"] = True
                    if all(required.values()):
                        break
            # Unsubscribe
            for msg in subscribe_msgs:
                unsub = msg.copy()
                unsub["op"] = "unsubscribe"
                await ws.send_json(unsub)
            return status

# --- /move helper: Publish movement command to ROSBridge ---
async def publish_move(cmd):
    async with aiohttp.ClientSession() as session:
        async with session.ws_connect(ROSBRIDGE_URI) as ws:
            msg = {
                "op": "publish",
                "topic": "/cmd_vel",
                "msg": cmd,
                "type": "geometry_msgs/Twist"
            }
            await ws.send_json(msg)
            # Optionally wait for ACK or just short delay
            await asyncio.sleep(0.1)
            return {"result": "success"}

# --- /task helper: Start/stop scripts via ROSBridge service call ---
async def manage_task(action, script_type):
    # This assumes existence of a ROSBridge service for script management
    service_name = f"/{script_type}_{action}"
    async with aiohttp.ClientSession() as session:
        async with session.ws_connect(ROSBRIDGE_URI) as ws:
            srv_msg = {
                "op": "call_service",
                "service": service_name,
                "args": {},
                "id": f"task_{action}_{script_type}"
            }
            await ws.send_json(srv_msg)
            async for msg in ws:
                resp = json.loads(msg.data)
                if resp.get("id") == srv_msg["id"]:
                    return resp.get("values", {"result": "unknown"})
    return {"result": "failed"}

# --- RTSP to HTTP MJPEG Streaming Proxy ---
async def mjpeg_stream(request):
    # OpenCV VideoCapture on RTSP stream
    cap = cv2.VideoCapture(RTSP_URL)
    if not cap.isOpened():
        return web.Response(status=503, text="Unable to connect to RTSP stream")
    async def stream_response(resp):
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            ret, jpeg = cv2.imencode('.jpg', frame)
            if not ret:
                break
            await resp.write(b"--frame\r\n")
            await resp.write(b"Content-Type: image/jpeg\r\n\r\n")
            await resp.write(jpeg.tobytes())
            await resp.write(b"\r\n")
            await asyncio.sleep(0.04)  # ~25FPS
        cap.release()
    headers = {
        'Content-Type': 'multipart/x-mixed-replace; boundary=frame'
    }
    resp = web.StreamResponse(status=200, reason='OK', headers=headers)
    await resp.prepare(request)
    await stream_response(resp)
    return resp

# --- API Handlers ---

async def status_handler(request):
    status = await fetch_status()
    return web.json_response(status)

async def move_handler(request):
    try:
        payload = await request.json()
        cmd = payload.get("cmd_vel") or payload.get("cmd_vel_corrected") or payload
        # Expect geometry_msgs/Twist format
        result = await publish_move(cmd)
        return web.json_response(result)
    except Exception as e:
        return web.json_response({"error": str(e)}, status=400)

async def task_handler(request):
    try:
        payload = await request.json()
        action = payload.get("action")  # "start" or "stop"
        script_type = payload.get("script_type")  # e.g., "slam", "lidar", "navigation"
        if action not in ("start", "stop") or not script_type:
            raise Exception("Invalid action or script_type")
        result = await manage_task(action, script_type)
        return web.json_response(result)
    except Exception as e:
        return web.json_response({"error": str(e)}, status=400)

# --- App Setup ---

app = web.Application()
app.router.add_route('GET', '/video', mjpeg_stream)
app.router.add_route('GET', '/status', status_handler)
app.router.add_route('POST', '/move', move_handler)
app.router.add_route('POST', '/task', task_handler)

if __name__ == "__main__":
    web.run_app(app, host=SERVER_HOST, port=SERVER_PORT)