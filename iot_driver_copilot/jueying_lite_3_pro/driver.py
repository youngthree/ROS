import os
import asyncio
import json
import aiohttp
from aiohttp import web
import websockets
import cv2
import numpy as np

# Environment Variables
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
RTSP_PORT = int(os.environ.get("RTSP_PORT", "554"))
ROS_API_PORT = int(os.environ.get("ROS_API_PORT", "9090"))  # For ROSBridge websocket
HTTP_SERVER_HOST = os.environ.get("HTTP_SERVER_HOST", "0.0.0.0")
HTTP_SERVER_PORT = int(os.environ.get("HTTP_SERVER_PORT", "8000"))
RTSP_PATH = os.environ.get("RTSP_PATH", "/stream1")
RTSP_USERNAME = os.environ.get("RTSP_USERNAME", "")
RTSP_PASSWORD = os.environ.get("RTSP_PASSWORD", "")

# Construct RTSP URL
if RTSP_USERNAME and RTSP_PASSWORD:
    RTSP_URL = f"rtsp://{RTSP_USERNAME}:{RTSP_PASSWORD}@{DEVICE_IP}:{RTSP_PORT}{RTSP_PATH}"
else:
    RTSP_URL = f"rtsp://{DEVICE_IP}:{RTSP_PORT}{RTSP_PATH}"

ROSBRIDGE_URL = f"ws://{DEVICE_IP}:{ROS_API_PORT}"

# ---- MJPEG Video Stream Handler ----
class VideoStream:
    def __init__(self, rtsp_url):
        self.rtsp_url = rtsp_url
        self.frame = None
        self.running = False
        self.capture = None

    def start(self):
        if not self.running:
            self.running = True
            self.capture = cv2.VideoCapture(self.rtsp_url)
            asyncio.create_task(self.update())

    async def update(self):
        while self.running:
            ret, frame = self.capture.read()
            if not ret:
                await asyncio.sleep(0.1)
                continue
            self.frame = frame
            await asyncio.sleep(0.03)  # ~30 FPS

    def get_jpeg(self):
        if self.frame is not None:
            ret, jpeg = cv2.imencode('.jpg', self.frame)
            if ret:
                return jpeg.tobytes()
        return None

    def stop(self):
        self.running = False
        if self.capture is not None:
            self.capture.release()

video_stream = VideoStream(RTSP_URL)

# ---- ROSBridge API Helper ----
async def rosbridge_call(service, args=None):
    async with websockets.connect(ROSBRIDGE_URL, ping_interval=None) as ws:
        request_id = "req" + os.urandom(4).hex()
        msg = {
            "op": "call_service",
            "service": service,
            "args": args or {},
            "id": request_id
        }
        await ws.send(json.dumps(msg))
        async for message in ws:
            resp = json.loads(message)
            if resp.get("id") == request_id and resp.get("result", False):
                return resp
            if resp.get("id") == request_id and "values" in resp:
                return resp
        return None

async def rosbridge_publish(topic, msg_type, data):
    async with websockets.connect(ROSBRIDGE_URL, ping_interval=None) as ws:
        msg = {
            "op": "publish",
            "topic": topic,
            "msg": data
        }
        await ws.send(json.dumps(msg))

async def rosbridge_subscribe_once(topic):
    async with websockets.connect(ROSBRIDGE_URL, ping_interval=None) as ws:
        sub_id = "sub" + os.urandom(4).hex()
        await ws.send(json.dumps({
            "op": "subscribe",
            "id": sub_id,
            "topic": topic,
            "queue_length": 1
        }))
        async for message in ws:
            resp = json.loads(message)
            if resp.get("op") == "publish" and resp.get("topic") == topic:
                return resp["msg"]

# ---- HTTP API Handlers ----
routes = web.RouteTableDef()

@routes.post('/task')
async def handle_task(request):
    """
    Manage operational scripts such as SLAM, LiDAR, or navigation.
    JSON Body: { "action": "start/stop", "script": "slam/lidar/navigation" }
    """
    try:
        body = await request.json()
        action = body.get("action")
        script = body.get("script")
        if action not in ["start", "stop"] or script not in ["slam", "lidar", "navigation"]:
            return web.json_response({"error": "Invalid action or script"}, status=400)
        # Example: publish to /slam_control, /lidar_control, /navigation_control
        topic = f"/{script}_control"
        await rosbridge_publish(topic, "std_msgs/String", {"data": action})
        return web.json_response({"status": "ok", "message": f"{action}ed {script}"})
    except Exception as e:
        return web.json_response({"error": str(e)}, status=500)

@routes.get('/status')
async def handle_status(request):
    """
    Fetch current status data including localization and navigation metrics.
    """
    try:
        # Example topics: /localization, /navigation_status, /imu, /joint_states
        localization = await rosbridge_subscribe_once("/localization")
        navigation = await rosbridge_subscribe_once("/navigation_status")
        imu = await rosbridge_subscribe_once("/imu")
        joint_states = await rosbridge_subscribe_once("/joint_states")
        return web.json_response({
            "localization": localization,
            "navigation": navigation,
            "imu": imu,
            "joint_states": joint_states
        })
    except Exception as e:
        return web.json_response({"error": str(e)}, status=500)

@routes.post('/move')
async def handle_move(request):
    """
    Send movement commands to the robot.
    JSON Body: { "linear": {"x": ..., "y": ..., "z": ...}, "angular": {"x": ..., "y": ..., "z": ...} }
    """
    try:
        body = await request.json()
        # Publish to /cmd_vel
        await rosbridge_publish("/cmd_vel", "geometry_msgs/Twist", body)
        return web.json_response({"status": "ok"})
    except Exception as e:
        return web.json_response({"error": str(e)}, status=500)

@routes.get('/video')
async def mjpeg_video(request):
    """
    MJPEG stream endpoint for RTSP video, consumable by browsers.
    """
    if not video_stream.running:
        video_stream.start()

    async def video_generator():
        boundary = "frame"
        while True:
            frame = video_stream.get_jpeg()
            if frame:
                yield (
                    b"--%b\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: %d\r\n\r\n" % (
                        boundary.encode(), len(frame)
                    )
                )
                yield frame
                yield b"\r\n"
            await asyncio.sleep(0.03)

    return web.Response(
        status=200,
        headers={
            "Content-Type": "multipart/x-mixed-replace; boundary=frame",
            "Cache-Control": "no-cache",
            "Connection": "close"
        },
        body=video_generator()
    )

# ---- App Runner ----
app = web.Application()
app.add_routes(routes)

if __name__ == "__main__":
    web.run_app(app, host=HTTP_SERVER_HOST, port=HTTP_SERVER_PORT)