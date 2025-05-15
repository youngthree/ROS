import os
import asyncio
import json
from aiohttp import web, ClientSession, WSMsgType
import aiohttp
import cv2
import numpy as np

# Environment Variables
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
ROS_API_BASE = os.environ.get("ROS_API_BASE", f"http://{DEVICE_IP}:5000")  # Example fallback
RTSP_URL = os.environ.get("RTSP_URL", f"rtsp://{DEVICE_IP}:8554/live")
HTTP_SERVER_HOST = os.environ.get("HTTP_SERVER_HOST", "0.0.0.0")
HTTP_SERVER_PORT = int(os.environ.get("HTTP_SERVER_PORT", "8080"))
RTSP_STREAM_FPS = int(os.environ.get("RTSP_STREAM_FPS", "10"))
RTSP_USER = os.environ.get("RTSP_USER", "")
RTSP_PASS = os.environ.get("RTSP_PASS", "")

# --------- Video Stream Proxy: RTSP to HTTP MJPEG ---------
# MJPEG Stream generator
async def mjpeg_stream_response(request):
    cap_args = RTSP_URL
    if RTSP_USER and RTSP_PASS:
        userinfo = f"{RTSP_USER}:{RTSP_PASS}@"
        cap_args = RTSP_URL.replace("rtsp://", f"rtsp://{userinfo}")
    cap = cv2.VideoCapture(cap_args)
    if not cap.isOpened():
        return web.Response(status=503, text="Could not connect to RTSP stream")

    async def mjpeg_stream_gen():
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    await asyncio.sleep(0.1)
                    continue
                ret, jpg = cv2.imencode('.jpg', frame)
                if not ret:
                    continue
                img_bytes = jpg.tobytes()
                boundary = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                yield boundary + img_bytes + b"\r\n"
                await asyncio.sleep(1.0 / RTSP_STREAM_FPS)
        finally:
            cap.release()

    headers = {
        "Content-Type": "multipart/x-mixed-replace; boundary=frame",
        "Cache-Control": "no-cache",
        "Pragma": "no-cache"
    }
    return web.Response(body=mjpeg_stream_gen(), status=200, headers=headers)

# --------- ROS/HTTP API Proxies ---------
async def start_stop_task(request):
    """
    POST /task
    JSON: { "action": "start"/"stop", "script": "SLAM"/"LiDAR"/"navigation" }
    """
    try:
        data = await request.json()
    except Exception:
        return web.json_response({"error": "Invalid JSON"}, status=400)
    async with ClientSession() as session:
        url = f"{ROS_API_BASE}/task"
        async with session.post(url, json=data) as resp:
            try:
                payload = await resp.json()
            except Exception:
                payload = await resp.text()
            return web.json_response(payload, status=resp.status)

async def get_status(request):
    """
    GET /status
    """
    async with ClientSession() as session:
        url = f"{ROS_API_BASE}/status"
        async with session.get(url) as resp:
            try:
                payload = await resp.json()
            except Exception:
                payload = await resp.text()
            return web.json_response(payload, status=resp.status)

async def move_robot(request):
    """
    POST /move
    JSON: { "linear": {...}, "angular": {...} }
    """
    try:
        data = await request.json()
    except Exception:
        return web.json_response({"error": "Invalid JSON"}, status=400)
    async with ClientSession() as session:
        url = f"{ROS_API_BASE}/move"
        async with session.post(url, json=data) as resp:
            try:
                payload = await resp.json()
            except Exception:
                payload = await resp.text()
            return web.json_response(payload, status=resp.status)

# --------- HTTP App Setup ---------
app = web.Application()
routes = [
    web.get('/video', mjpeg_stream_response),
    web.post('/task', start_stop_task),
    web.get('/status', get_status),
    web.post('/move', move_robot)
]
app.add_routes(routes)

if __name__ == "__main__":
    web.run_app(app, host=HTTP_SERVER_HOST, port=HTTP_SERVER_PORT)