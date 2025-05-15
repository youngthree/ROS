import os
import asyncio
import json
from aiohttp import web
import aiohttp
import base64

import cv2
import numpy as np

# Env vars
DEVICE_IP = os.environ.get('DEVICE_IP', '127.0.0.1')
ROSBRIDGE_PORT = int(os.environ.get('ROSBRIDGE_PORT', '9090'))
RTSP_URL = os.environ.get('RTSP_URL', f'rtsp://{DEVICE_IP}:8554/live')
HTTP_SERVER_HOST = os.environ.get('HTTP_SERVER_HOST', '0.0.0.0')
HTTP_SERVER_PORT = int(os.environ.get('HTTP_SERVER_PORT', '8000'))

# --- ROSBridge JSON API ---
# ROSBridge websocket API for publishing/subscribing to ROS topics
try:
    import websockets
except ImportError:
    websockets = None

# --- Video Streaming Helper ---
class MJPEGStreamer:
    def __init__(self, rtsp_url):
        self.rtsp_url = rtsp_url
        self.frame = None
        self.running = False

    async def start(self):
        self.running = True
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(None, self._capture_loop)

    def _capture_loop(self):
        cap = cv2.VideoCapture(self.rtsp_url)
        if not cap.isOpened():
            self.running = False
            return
        while self.running:
            ret, frame = cap.read()
            if not ret:
                continue
            # Resize for browser performance
            frame = cv2.resize(frame, (640, 360))
            ret, jpeg = cv2.imencode('.jpg', frame)
            if ret:
                self.frame = jpeg.tobytes()
        cap.release()

    def get_frame(self):
        return self.frame

    def stop(self):
        self.running = False

mjpeg_streamer = MJPEGStreamer(RTSP_URL)

# --- ROSBridge Helper ---
class ROSBridgeClient:
    def __init__(self, ip, port):
        self.uri = f'ws://{ip}:{port}'
        self.ws = None
        self.subscriptions = {}
        self.status_data = {}
        self.connected = False

    async def connect(self):
        if websockets is None:
            return
        self.ws = await websockets.connect(self.uri)
        self.connected = True
        asyncio.create_task(self.receive_loop())

    async def receive_loop(self):
        if not self.ws:
            return
        try:
            async for msg in self.ws:
                data = json.loads(msg)
                if 'topic' in data:
                    # Save status data from subscriptions
                    self.status_data[data['topic']] = data['msg']
        except Exception:
            self.connected = False

    async def subscribe(self, topic, msg_type):
        if not self.connected:
            await self.connect()
        sub_id = f"sub_{topic}"
        sub_msg = {
            "op": "subscribe",
            "id": sub_id,
            "topic": topic,
            "type": msg_type,
            "throttle_rate": 100
        }
        await self.ws.send(json.dumps(sub_msg))
        self.subscriptions[topic] = msg_type

    async def publish(self, topic, msg_type, msg):
        if not self.connected:
            await self.connect()
        pub_id = f"pub_{topic}"
        pub_msg = {
            "op": "publish",
            "id": pub_id,
            "topic": topic,
            "type": msg_type,
            "msg": msg
        }
        await self.ws.send(json.dumps(pub_msg))

    def get_status(self):
        # Return collected status data
        return self.status_data

ros_client = ROSBridgeClient(DEVICE_IP, ROSBRIDGE_PORT)

# --- API Handlers ---
async def video_mjpeg(request):
    async def mjpeg_response_gen():
        boundary = '--frame'
        while True:
            frame = mjpeg_streamer.get_frame()
            if frame is not None:
                yield (
                    f"{boundary}\r\n"
                    "Content-Type: image/jpeg\r\n"
                    f"Content-Length: {len(frame)}\r\n"
                    "\r\n"
                ).encode('utf-8') + frame + b"\r\n"
            await asyncio.sleep(0.05)
    headers = {
        'Content-Type': 'multipart/x-mixed-replace; boundary=frame'
    }
    return web.Response(body=mjpeg_response_gen(), headers=headers)

async def post_task(request):
    payload = await request.json()
    # Example: {"action": "start", "script": "slam"}
    action = payload.get('action')
    script = payload.get('script')
    if not action or not script:
        return web.json_response({'error': 'action and script required'}, status=400)
    # Map script to ROS services/topics
    # For demo, we just publish to a topic
    if action == 'start':
        msg = {"data": f"start_{script}"}
    else:
        msg = {"data": f"stop_{script}"}
    await ros_client.publish("/robot/script_control", "std_msgs/String", msg)
    return web.json_response({'result': f'{action} {script} request sent'})

async def get_status(request):
    # Subscribe to relevant topics if not already
    topics = [
        ("/odom", "nav_msgs/Odometry"),
        ("/imu", "sensor_msgs/Imu"),
        ("/joint_states", "sensor_msgs/JointState"),
        ("/ultrasound_distance", "std_msgs/Float32"),
        ("/localization_status", "std_msgs/String"),
        ("/navigation_status", "std_msgs/String"),
        ("/yolov8/detections", "std_msgs/String")
    ]
    # Subscribe if not already
    for topic, msg_type in topics:
        if topic not in ros_client.subscriptions:
            await ros_client.subscribe(topic, msg_type)
    await asyncio.sleep(0.2)
    data = ros_client.get_status()
    return web.json_response(data)

async def post_move(request):
    payload = await request.json()
    # Example: {"linear": {"x": 1.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
    linear = payload.get('linear', {})
    angular = payload.get('angular', {})
    msg = {
        "linear": {k: float(linear.get(k, 0.0)) for k in ("x", "y", "z")},
        "angular": {k: float(angular.get(k, 0.0)) for k in ("x", "y", "z")}
    }
    await ros_client.publish("/cmd_vel", "geometry_msgs/Twist", msg)
    return web.json_response({'result': 'cmd_vel sent'})

# --- App Setup ---
async def on_startup(app):
    asyncio.create_task(mjpeg_streamer.start())
    asyncio.create_task(ros_client.connect())

async def on_cleanup(app):
    mjpeg_streamer.stop()
    if ros_client.ws is not None:
        await ros_client.ws.close()

app = web.Application()
app.router.add_get('/video', video_mjpeg)
app.router.add_post('/task', post_task)
app.router.add_get('/status', get_status)
app.router.add_post('/move', post_move)
app.on_startup.append(on_startup)
app.on_cleanup.append(on_cleanup)

if __name__ == '__main__':
    web.run_app(app, host=HTTP_SERVER_HOST, port=HTTP_SERVER_PORT)