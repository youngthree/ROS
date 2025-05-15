import os
import asyncio
import json
import base64
import aiohttp
import aiohttp.web
import websockets
import cv2
import numpy as np

from aiohttp import web

# Environment Variables with defaults for development
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
ROSBRIDGE_WS_PORT = int(os.environ.get("ROSBRIDGE_WS_PORT", 9090))
HTTP_HOST = os.environ.get("HTTP_HOST", "0.0.0.0")
HTTP_PORT = int(os.environ.get("HTTP_PORT", 8080))
RTSP_STREAM_URL = os.environ.get("RTSP_STREAM_URL", f"rtsp://{DEVICE_IP}:8554/live")
RTSP_FPS = int(os.environ.get("RTSP_FPS", 10))

# --- Helper Functions ---

async def rosbridge_call(service, args=None, op="call_service"):
    uri = f"ws://{DEVICE_IP}:{ROSBRIDGE_WS_PORT}"
    async with websockets.connect(uri) as ws:
        msg = {"op": op, "service": service}
        if args:
            msg["args"] = args
        await ws.send(json.dumps(msg))
        response = await ws.recv()
        return json.loads(response)

async def rosbridge_publish(topic, msg_type, msg):
    uri = f"ws://{DEVICE_IP}:{ROSBRIDGE_WS_PORT}"
    async with websockets.connect(uri) as ws:
        pub = {
            "op": "publish",
            "topic": topic,
            "msg": msg
        }
        await ws.send(json.dumps(pub))
        # Optionally wait for ack, but ROSBridge doesn't always send one

async def rosbridge_subscribe(topic, msg_type, timeout=2.0):
    uri = f"ws://{DEVICE_IP}:{ROSBRIDGE_WS_PORT}"
    async with websockets.connect(uri) as ws:
        sub = {
            "op": "subscribe",
            "topic": topic,
            "type": msg_type,
            "queue_length": 1
        }
        await ws.send(json.dumps(sub))
        try:
            msg = await asyncio.wait_for(ws.recv(), timeout=timeout)
            return json.loads(msg)
        except asyncio.TimeoutError:
            return None

# --- HTTP Handlers ---

async def handle_status(request):
    # Collect sensor and status data
    localization = await rosbridge_subscribe("/localization", "nav_msgs/Odometry")
    navigation = await rosbridge_subscribe("/navigation_status", "std_msgs/String")
    imu = await rosbridge_subscribe("/imu/data", "sensor_msgs/Imu")
    joint = await rosbridge_subscribe("/joint_states", "sensor_msgs/JointState")
    odom = await rosbridge_subscribe("/odom", "nav_msgs/Odometry")
    # Aggregate results
    result = {
        "localization": localization,
        "navigation_status": navigation,
        "imu": imu,
        "joint_states": joint,
        "odom": odom
    }
    return web.json_response(result)

async def handle_move(request):
    try:
        body = await request.json()
        # Expecting either 'cmd_vel' or 'cmd_vel_corrected' (geometry_msgs/Twist)
        if "cmd_vel_corrected" in body:
            topic = "/cmd_vel_corrected"
            msg = body["cmd_vel_corrected"]
        else:
            topic = "/cmd_vel"
            msg = body.get("cmd_vel", body)
        await rosbridge_publish(topic, "geometry_msgs/Twist", msg)
        return web.json_response({"success": True})
    except Exception as e:
        return web.json_response({"success": False, "error": str(e)}, status=400)

async def handle_task(request):
    # POST: {"action": "start"|"stop", "script_type": "slam"|"lidar"|"navigation"}
    try:
        body = await request.json()
        action = body.get("action")
        script_type = body.get("script_type")
        if not action or not script_type:
            raise ValueError("Missing action or script_type")
        # Use a topic for script management, e.g. /script_control (std_msgs/String)
        msg = {"data": f"{action}:{script_type}"}
        await rosbridge_publish("/script_control", "std_msgs/String", msg)
        return web.json_response({"success": True})
    except Exception as e:
        return web.json_response({"success": False, "error": str(e)}, status=400)

# --- RTSP to HTTP MJPEG Proxy ---

async def mjpeg_proxy(request):
    # Stream RTSP video as multipart/x-mixed-replace MJPEG stream
    boundary = "frame"
    response = web.StreamResponse(
        status=200,
        reason="OK",
        headers={
            "Content-Type": f"multipart/x-mixed-replace; boundary={boundary}"
        }
    )
    await response.prepare(request)
    cap = cv2.VideoCapture(RTSP_STREAM_URL)
    if not cap.isOpened():
        await response.write(b"Failed to open RTSP stream.\n")
        await response.write_eof()
        return response
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                await asyncio.sleep(0.2)
                continue
            # Encode frame as JPEG
            ret, jpeg = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            data = jpeg.tobytes()
            part = (b"--" + boundary.encode() + b"\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(data)).encode() + b"\r\n\r\n" +
                    data + b"\r\n")
            await response.write(part)
            await asyncio.sleep(1.0 / RTSP_FPS)
    except asyncio.CancelledError:
        pass
    finally:
        cap.release()
        await response.write_eof()
    return response

# --- App Setup ---

app = web.Application()
app.router.add_get("/status", handle_status)
app.router.add_post("/move", handle_move)
app.router.add_post("/task", handle_task)
app.router.add_get("/video", mjpeg_proxy)  # HTTP MJPEG stream of RTSP

if __name__ == "__main__":
    web.run_app(app, host=HTTP_HOST, port=HTTP_PORT)