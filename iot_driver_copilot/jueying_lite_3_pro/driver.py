import os
import io
import json
import asyncio
import struct
import base64
from aiohttp import web
import aiohttp
import cv2
import numpy as np

# ------------------- Environment Configuration -------------------

DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
DEVICE_ROSBRIDGE_PORT = int(os.environ.get("DEVICE_ROSBRIDGE_PORT", "9090"))
DEVICE_RTSP_PORT = int(os.environ.get("DEVICE_RTSP_PORT", "554"))
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", "8080"))
RTSP_PATH = os.environ.get("DEVICE_RTSP_PATH", "/live")  # e.g., "/live"
RTSP_USERNAME = os.environ.get("DEVICE_RTSP_USERNAME", None)
RTSP_PASSWORD = os.environ.get("DEVICE_RTSP_PASSWORD", None)

# ------------------- ROSBridge Minimal WebSocket Client -------------------

class ROSBridgeClient:
    def __init__(self, ip, port):
        self.uri = f"ws://{ip}:{port}"
        self.session = None
        self.ws = None
        self._lock = asyncio.Lock()

    async def connect(self):
        if self.ws is not None and not self.ws.closed:
            return
        if self.session is None:
            self.session = aiohttp.ClientSession()
        self.ws = await self.session.ws_connect(self.uri)

    async def close(self):
        if self.ws is not None:
            await self.ws.close()
        if self.session is not None:
            await self.session.close()

    async def call_service(self, service, args=None):
        await self.connect()
        srv_id = "call_service_" + str(np.random.randint(0,999999))
        req = {
            "op": "call_service",
            "service": service,
            "args": args or {},
            "id": srv_id
        }
        await self.ws.send_json(req)
        async for msg in self.ws:
            if msg.type == aiohttp.WSMsgType.TEXT:
                resp = json.loads(msg.data)
                if resp.get("id") == srv_id:
                    return resp
            elif msg.type == aiohttp.WSMsgType.CLOSED:
                break

    async def publish(self, topic, msg_type, msg):
        await self.connect()
        req = {
            "op": "publish",
            "topic": topic,
            "msg": msg,
            "type": msg_type,
        }
        await self.ws.send_json(req)

    async def subscribe_once(self, topic, msg_type, wait_timeout=1.0):
        await self.connect()
        sub_id = "sub_once_" + str(np.random.randint(0,999999))
        req = {
            "op": "subscribe",
            "topic": topic,
            "type": msg_type,
            "id": sub_id
        }
        await self.ws.send_json(req)
        try:
            async with asyncio.timeout(wait_timeout):
                async for msg in self.ws:
                    if msg.type == aiohttp.WSMsgType.TEXT:
                        resp = json.loads(msg.data)
                        if resp.get("topic") == topic and resp.get("msg"):
                            unsub_req = {
                                "op": "unsubscribe",
                                "topic": topic,
                                "id": sub_id
                            }
                            await self.ws.send_json(unsub_req)
                            return resp["msg"]
                    elif msg.type == aiohttp.WSMsgType.CLOSED:
                        break
        except asyncio.TimeoutError:
            return None

# ------------------- RTSP to HTTP MJPEG Proxy -------------------

class RTSPMJPEGProxy:
    def __init__(self, ip, port, path, username=None, password=None):
        self.rtsp_url = self.build_rtsp_url(ip, port, path, username, password)
        self.cap = None

    @staticmethod
    def build_rtsp_url(ip, port, path, username, password):
        if username and password:
            return f"rtsp://{username}:{password}@{ip}:{port}{path}"
        else:
            return f"rtsp://{ip}:{port}{path}"

    def open(self):
        if self.cap is None or not self.cap.isOpened():
            self.cap = cv2.VideoCapture(self.rtsp_url)

    def close(self):
        if self.cap:
            self.cap.release()
            self.cap = None

    def read_frame(self):
        self.open()
        if self.cap is None or not self.cap.isOpened():
            return None
        ret, frame = self.cap.read()
        if not ret:
            return None
        ret, buf = cv2.imencode('.jpg', frame)
        if not ret:
            return None
        return buf.tobytes()

    async def mjpeg_stream(self):
        boundary = "frame"
        while True:
            frame = self.read_frame()
            if frame is None:
                await asyncio.sleep(0.1)
                continue
            yield (b"--" + boundary.encode() + b"\r\n"
                   b"Content-Type: image/jpeg\r\n"
                   b"Content-Length: " + str(len(frame)).encode() + b"\r\n\r\n" +
                   frame + b"\r\n")
            await asyncio.sleep(0.04) # ~25fps

# ------------------- HTTP API Handlers -------------------

rosbridge = ROSBridgeClient(DEVICE_IP, DEVICE_ROSBRIDGE_PORT)
rtsp_proxy = RTSPMJPEGProxy(DEVICE_IP, DEVICE_RTSP_PORT, RTSP_PATH, RTSP_USERNAME, RTSP_PASSWORD)

routes = web.RouteTableDef()

@routes.post('/task')
async def manage_task(request):
    # Expect JSON: {"action": "start"/"stop", "script": "SLAM"/"LiDAR"/"navigation"}
    data = await request.json()
    action = data.get("action")
    script = data.get("script")
    # This is a placeholder mapping. Adjust topic/service as per your robot's config.
    topic = "/jueying/task_control"
    msg_type = "std_msgs/String"
    msg = {"data": f"{action}:{script}"}
    await rosbridge.publish(topic, msg_type, msg)
    return web.json_response({"status": "sent", "action": action, "script": script})

@routes.get('/status')
async def get_status(request):
    # Collect a selection of status topics
    odom = await rosbridge.subscribe_once("/leg_odom", "nav_msgs/Odometry")
    imu = await rosbridge.subscribe_once("/imu_data", "sensor_msgs/Imu")
    handle_state = await rosbridge.subscribe_once("/handle_state", "std_msgs/String")
    nav_status = await rosbridge.subscribe_once("/navigation_status", "std_msgs/String")
    result = {
        "leg_odom": odom,
        "imu_data": imu,
        "handle_state": handle_state,
        "navigation_status": nav_status
    }
    return web.json_response(result)

@routes.post('/move')
async def move_robot(request):
    # Expect JSON: {"linear": {"x":..,"y":..,"z":..}, "angular": {"x":..,"y":..,"z":..}}
    data = await request.json()
    topic = "/cmd_vel"
    msg_type = "geometry_msgs/Twist"
    msg = {
        "linear": data.get("linear", {"x":0,"y":0,"z":0}),
        "angular": data.get("angular", {"x":0,"y":0,"z":0})
    }
    await rosbridge.publish(topic, msg_type, msg)
    return web.json_response({"status": "sent", "requested": msg})

@routes.get('/video')
async def video_stream(request):
    boundary = "frame"
    response = web.StreamResponse(
        status=200,
        reason='OK',
        headers={
            'Content-Type': f'multipart/x-mixed-replace; boundary={boundary}'
        }
    )
    await response.prepare(request)
    async for frame in rtsp_proxy.mjpeg_stream():
        await response.write(frame)
    return response

@routes.get('/')
async def index(request):
    html = """
    <html>
    <head><title>Jueying Lite3 Pro Driver</title></head>
    <body>
        <h1>Jueying Lite3 Pro HTTP Driver</h1>
        <ul>
            <li>POST /task - Manage operational scripts</li>
            <li>GET /status - Get current status</li>
            <li>POST /move - Move robot</li>
            <li>GET /video - Live video (MJPEG stream)</li>
        </ul>
        <img src="/video" width="640"/>
    </body>
    </html>
    """
    return web.Response(text=html, content_type='text/html')

# ------------------- App Runner -------------------

app = web.Application()
app.add_routes(routes)

if __name__ == '__main__':
    web.run_app(app, host=SERVER_HOST, port=SERVER_PORT)