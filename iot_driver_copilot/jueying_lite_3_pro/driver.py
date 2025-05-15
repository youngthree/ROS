import os
import json
import asyncio
from aiohttp import web
import aiohttp
import cv2
import numpy as np

# Environment variables for configuration
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", "8000"))
RTSP_PORT = int(os.environ.get("RTSP_PORT", "554"))  # Used only for RTSP video stream
ROS_API_URL = os.environ.get("ROS_API_URL", f"http://{DEVICE_IP}:5000")  # Assumed ROS bridge/web API

RTSP_STREAM_PATH = os.environ.get("RTSP_STREAM_PATH", "/stream1")
RTSP_USERNAME = os.environ.get("RTSP_USERNAME", "")
RTSP_PASSWORD = os.environ.get("RTSP_PASSWORD", "")

# Helper: build RTSP URL
def rtsp_url():
    creds = ""
    if RTSP_USERNAME and RTSP_PASSWORD:
        creds = f"{RTSP_USERNAME}:{RTSP_PASSWORD}@"
    return f"rtsp://{creds}{DEVICE_IP}:{RTSP_PORT}{RTSP_STREAM_PATH}"

# MJPEG streaming generator
async def mjpeg_stream(request):
    # OpenCV VideoCapture for RTSP
    cap = cv2.VideoCapture(rtsp_url())
    if not cap.isOpened():
        return web.Response(status=503, text="Unable to open RTSP stream")

    async def stream_response(resp):
        try:
            while True:
                ret, img = cap.read()
                if not ret:
                    break
                _, jpeg = cv2.imencode('.jpg', img)
                frame = jpeg.tobytes()
                await resp.write(
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                )
                await asyncio.sleep(0.04)  # ~25 fps
        finally:
            cap.release()

    resp = web.StreamResponse(
        status=200,
        reason='OK',
        headers={
            'Content-Type': 'multipart/x-mixed-replace; boundary=frame'
        }
    )
    await resp.prepare(request)
    await stream_response(resp)
    return resp

# ROS API proxy helpers
async def ros_post(path, json_data):
    url = f"{ROS_API_URL}{path}"
    async with aiohttp.ClientSession() as session:
        async with session.post(url, json=json_data) as resp:
            data = await resp.text()
            try:
                return web.json_response(json.loads(data), status=resp.status)
            except Exception:
                return web.Response(text=data, status=resp.status)

async def ros_get(path):
    url = f"{ROS_API_URL}{path}"
    async with aiohttp.ClientSession() as session:
        async with session.get(url) as resp:
            data = await resp.text()
            try:
                return web.json_response(json.loads(data), status=resp.status)
            except Exception:
                return web.Response(text=data, status=resp.status)

# API endpoints
async def api_task(request):
    # POST /task: { "action": "start"/"stop", "script": "slam"/"lidar"/"navigation" }
    try:
        payload = await request.json()
    except Exception:
        return web.Response(status=400, text="Invalid JSON")
    # Forward to ROS API or handle as needed
    return await ros_post("/task", payload)

async def api_status(request):
    # GET /status: returns current status data
    return await ros_get("/status")

async def api_move(request):
    # POST /move: { "cmd_vel": {...}, ... }
    try:
        payload = await request.json()
    except Exception:
        return web.Response(status=400, text="Invalid JSON")
    return await ros_post("/move", payload)

# HTTP server app
app = web.Application()
app.router.add_get('/video', mjpeg_stream)  # HTTP MJPEG video stream
app.router.add_post('/task', api_task)
app.router.add_get('/status', api_status)
app.router.add_post('/move', api_move)

if __name__ == "__main__":
    web.run_app(app, host=SERVER_HOST, port=SERVER_PORT)