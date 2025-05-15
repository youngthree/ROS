import os
import asyncio
import json
import base64
import aiohttp
from aiohttp import web
import cv2
import numpy as np

# Environment variables
DEVICE_IP = os.getenv("DEVICE_IP", "127.0.0.1")
ROS_API_URL = os.getenv("ROS_API_URL", f"http://{DEVICE_IP}:5000")  # Example ROS REST bridge
RTSP_URL = os.getenv("RTSP_URL", f"rtsp://{DEVICE_IP}/live")
SERVER_HOST = os.getenv("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.getenv("SERVER_PORT", "8080"))
RTSP_HTTP_STREAM_PATH = os.getenv("RTSP_HTTP_STREAM_PATH", "/video")  # Endpoint to serve HTTP MJPEG

# --- ROS Bridge Functions (Assume REST bridge on device or accessible) ---

async def ros_post(endpoint, data):
    url = f"{ROS_API_URL}{endpoint}"
    async with aiohttp.ClientSession() as session:
        async with session.post(url, json=data) as resp:
            result = await resp.text()
            try:
                return json.loads(result)
            except Exception:
                return {"result": result}

async def ros_get(endpoint):
    url = f"{ROS_API_URL}{endpoint}"
    async with aiohttp.ClientSession() as session:
        async with session.get(url) as resp:
            result = await resp.text()
            try:
                return json.loads(result)
            except Exception:
                return {"result": result}

# --- HTTP API Handlers ---

async def handle_task(request):
    try:
        data = await request.json()
        action = data.get("action")
        script_type = data.get("script_type")
        if not action or not script_type:
            return web.json_response({"error": "Missing action or script_type"}, status=400)
        # Call ROS REST API to start/stop scripts (e.g., /scripts)
        ros_resp = await ros_post("/scripts", {"action": action, "type": script_type})
        return web.json_response(ros_resp)
    except Exception as e:
        return web.json_response({"error": str(e)}, status=500)

async def handle_status(request):
    try:
        ros_resp = await ros_get("/status")
        return web.json_response(ros_resp)
    except Exception as e:
        return web.json_response({"error": str(e)}, status=500)

async def handle_move(request):
    try:
        data = await request.json()
        # Forward movement command to ROS
        ros_resp = await ros_post("/move", data)
        return web.json_response(ros_resp)
    except Exception as e:
        return web.json_response({"error": str(e)}, status=500)

# --- RTSP to HTTP MJPEG Proxy ---

class RTSPtoHTTPStream:
    def __init__(self, rtsp_url):
        self.rtsp_url = rtsp_url
        self.cap = None
        self.running = False

    def open(self):
        if not self.cap:
            self.cap = cv2.VideoCapture(self.rtsp_url)
            if not self.cap.isOpened():
                self.cap.release()
                self.cap = None

    def close(self):
        if self.cap:
            self.cap.release()
            self.cap = None

    def get_frame(self):
        if not self.cap:
            self.open()
        if self.cap:
            ret, frame = self.cap.read()
            if ret:
                ret, jpeg = cv2.imencode('.jpg', frame)
                if ret:
                    return jpeg.tobytes()
        return None

    async def mjpeg_generator(self):
        self.open()
        while True:
            frame = self.get_frame()
            if frame is None:
                await asyncio.sleep(0.1)
                continue
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            await asyncio.sleep(0.04)  # ~25fps

rtsp_streamer = RTSPtoHTTPStream(RTSP_URL)

async def handle_video(request):
    response = web.StreamResponse(
        status=200,
        reason='OK',
        headers={'Content-Type': 'multipart/x-mixed-replace; boundary=frame'}
    )
    await response.prepare(request)
    async for frame in rtsp_streamer.mjpeg_generator():
        await response.write(frame)
    return response

# --- App Factory ---

def make_app():
    app = web.Application()
    app.router.add_post('/task', handle_task)
    app.router.add_get('/status', handle_status)
    app.router.add_post('/move', handle_move)
    app.router.add_get(RTSP_HTTP_STREAM_PATH, handle_video)
    return app

# --- Main Entrypoint ---

if __name__ == '__main__':
    web.run_app(make_app(), host=SERVER_HOST, port=SERVER_PORT)