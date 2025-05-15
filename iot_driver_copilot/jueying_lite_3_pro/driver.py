import os
import asyncio
import json
import aiohttp
import aiohttp.web
import base64

import cv2
import numpy as np

from typing import Dict, Any

# Configuration via environment variables
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", "8080"))
RTSP_PORT = int(os.environ.get("RTSP_PORT", "8554"))
RTSP_PATH = os.environ.get("RTSP_PATH", "stream")
RTSP_USERNAME = os.environ.get("RTSP_USERNAME", "")
RTSP_PASSWORD = os.environ.get("RTSP_PASSWORD", "")
ROSBRIDGE_PORT = int(os.environ.get("ROSBRIDGE_PORT", "9090"))

# RTSP URL for camera stream
def get_rtsp_url():
    auth = ""
    if RTSP_USERNAME and RTSP_PASSWORD:
        auth = f"{RTSP_USERNAME}:{RTSP_PASSWORD}@"
    return f"rtsp://{auth}{DEVICE_IP}:{RTSP_PORT}/{RTSP_PATH}"

# --- ROSBRIDGE JSON API for topics (status, move, task) ---

class RosbridgeClient:
    def __init__(self, ip: str, port: int):
        self.url = f"ws://{ip}:{port}"
        self.session = None
        self.ws = None
        self.id_counter = 1
        self.pending = {}

    async def connect(self):
        if self.ws is None or self.ws.closed:
            self.session = aiohttp.ClientSession()
            self.ws = await self.session.ws_connect(self.url)

    async def close(self):
        if self.ws:
            await self.ws.close()
            self.ws = None
        if self.session:
            await self.session.close()
            self.session = None

    async def call_service(self, service: str, args: Dict[str, Any]):
        await self.connect()
        msg_id = str(self.id_counter)
        self.id_counter += 1
        request = {
            "op": "call_service",
            "service": service,
            "args": args,
            "id": msg_id
        }
        fut = asyncio.get_event_loop().create_future()
        self.pending[msg_id] = fut
        await self.ws.send_json(request)
        while True:
            msg = await self.ws.receive()
            if msg.type == aiohttp.WSMsgType.TEXT:
                data = json.loads(msg.data)
                if data.get("id") == msg_id and data.get("result", False):
                    fut.set_result(data.get("values", {}))
                    break
        return await fut

    async def publish(self, topic: str, msg: Dict[str, Any]):
        await self.connect()
        await self.ws.send_json({"op": "publish", "topic": topic, "msg": msg})

    async def subscribe_once(self, topic: str):
        await self.connect()
        msg_id = str(self.id_counter)
        self.id_counter += 1
        await self.ws.send_json({"op": "subscribe", "topic": topic, "id": msg_id})
        while True:
            msg = await self.ws.receive()
            if msg.type == aiohttp.WSMsgType.TEXT:
                data = json.loads(msg.data)
                if data.get("topic") == topic and "msg" in data:
                    # Unsubscribe after first message
                    await self.ws.send_json({"op": "unsubscribe", "topic": topic, "id": msg_id})
                    return data["msg"]

ros = RosbridgeClient(DEVICE_IP, ROSBRIDGE_PORT)

# --- HTTP Server Handlers ---

routes = aiohttp.web.RouteTableDef()

# /status: Fetch localization, navigation, sensor readings
@routes.get('/status')
async def status(request):
    # For demo, subscribe to /odom and /imu for odometry and imu data.
    # You can extend with more topics as needed.
    odom = await ros.subscribe_once("/odom")
    imu = await ros.subscribe_once("/imu")
    status = {
        "odom": odom,
        "imu": imu
    }
    return aiohttp.web.json_response(status)

# /move: Send velocity command (Twist) to /cmd_vel
@routes.post('/move')
async def move(request):
    try:
        body = await request.json()
    except Exception:
        return aiohttp.web.Response(status=400, text="Invalid JSON")
    # Expects geometry_msgs/Twist format
    await ros.publish("/cmd_vel", body)
    return aiohttp.web.json_response({"result": "ok"})

# /task: Start/Stop SLAM, LiDAR, or navigation scripts via a ROS service
@routes.post('/task')
async def task(request):
    try:
        body = await request.json()
        action = body.get("action")
        script_type = body.get("type")
    except Exception:
        return aiohttp.web.Response(status=400, text="Invalid JSON")
    # Assumes a ROS service for managing scripts, e.g., /manage_script
    # You may need to adapt service name and args to your robot's implementation
    result = await ros.call_service("/manage_script", {"action": action, "type": script_type})
    return aiohttp.web.json_response({"result": result})

# /video: HTTP MJPEG stream from RTSP
@routes.get('/video')
async def mjpeg_stream(request):
    async def stream_response(resp):
        cap = cv2.VideoCapture(get_rtsp_url())
        if not cap.isOpened():
            await resp.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
            await resp.write(b"")  # Write empty frame to trigger error in browser
            return
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                ret, jpeg = cv2.imencode('.jpg', frame)
                if not ret:
                    continue
                img_bytes = jpeg.tobytes()
                await resp.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + img_bytes + b"\r\n")
                await asyncio.sleep(0.03)  # ~30 fps
        finally:
            cap.release()
    headers = {
        'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
        'Cache-Control': 'no-cache',
        'Pragma': 'no-cache'
    }
    resp = aiohttp.web.StreamResponse(status=200, reason='OK', headers=headers)
    await resp.prepare(request)
    await stream_response(resp)
    return resp

# --- App Setup ---

app = aiohttp.web.Application()
app.add_routes(routes)

async def on_shutdown(app):
    await ros.close()

app.on_shutdown.append(on_shutdown)

if __name__ == "__main__":
    aiohttp.web.run_app(app, host=SERVER_HOST, port=SERVER_PORT)