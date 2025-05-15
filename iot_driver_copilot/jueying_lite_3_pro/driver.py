import os
import asyncio
import json
from aiohttp import web
import aiohttp
import cv2
import numpy as np

# ==== Environment Variables ====
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
ROS_API_URL = os.environ.get("ROS_API_URL", f"http://{DEVICE_IP}:5000")
RTSP_URL = os.environ.get("RTSP_URL", f"rtsp://{DEVICE_IP}/live")
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", "8080"))
VIDEO_STREAM_PATH = os.environ.get("VIDEO_STREAM_PATH", "/video")
STATUS_PATH = os.environ.get("STATUS_PATH", "/status")
MOVE_PATH = os.environ.get("MOVE_PATH", "/move")
TASK_PATH = os.environ.get("TASK_PATH", "/task")
# Only use RTSP port if needed, driver does not open any RTSP port, just connects out.

# ========== HTTP API ==========
routes = web.RouteTableDef()

# ---- Video Streaming Endpoint (RTSP to HTTP) ----
@routes.get(VIDEO_STREAM_PATH)
async def video_stream(request):
    async def stream_response(writer):
        # OpenCV VideoCapture for RTSP stream
        cap = cv2.VideoCapture(RTSP_URL)
        if not cap.isOpened():
            await writer.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
            await writer.write(b"Camera not available.\r\n")
            return
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                # Encode frame as JPEG
                ret2, buffer = cv2.imencode('.jpg', frame)
                if not ret2:
                    continue
                jpg_bytes = buffer.tobytes()
                await writer.write(
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + jpg_bytes + b"\r\n"
                )
                await writer.drain()
                await asyncio.sleep(0.04)  # ~25 FPS
        finally:
            cap.release()
    headers = {
        "Content-Type": "multipart/x-mixed-replace; boundary=frame"
    }
    return web.Response(body=stream_response, headers=headers)

# ---- Status Endpoint (GET) ----
@routes.get(STATUS_PATH)
async def get_status(request):
    async with aiohttp.ClientSession() as session:
        try:
            async with session.get(f"{ROS_API_URL}/status") as resp:
                data = await resp.json()
                return web.json_response(data)
        except Exception as e:
            return web.json_response({"error": str(e)}, status=500)

# ---- Move Endpoint (POST) ----
@routes.post(MOVE_PATH)
async def move_robot(request):
    try:
        payload = await request.json()
    except:
        return web.json_response({"error": "Invalid JSON"}, status=400)
    async with aiohttp.ClientSession() as session:
        try:
            async with session.post(f"{ROS_API_URL}/move", json=payload) as resp:
                data = await resp.json()
                return web.json_response(data)
        except Exception as e:
            return web.json_response({"error": str(e)}, status=500)

# ---- Task Endpoint (POST) ----
@routes.post(TASK_PATH)
async def manage_task(request):
    try:
        payload = await request.json()
    except:
        return web.json_response({"error": "Invalid JSON"}, status=400)
    async with aiohttp.ClientSession() as session:
        try:
            async with session.post(f"{ROS_API_URL}/task", json=payload) as resp:
                data = await resp.json()
                return web.json_response(data)
        except Exception as e:
            return web.json_response({"error": str(e)}, status=500)

# ---- Root Endpoint ----
@routes.get("/")
async def index(request):
    return web.Response(text=f"""
    <html>
    <head><title>Jueying Lite3 Pro Robot Driver</title></head>
    <body>
        <h2>Jueying Lite3 Pro HTTP Driver</h2>
        <ul>
            <li><a href="{VIDEO_STREAM_PATH}">Live Video Stream (in browser)</a></li>
            <li><a href="{STATUS_PATH}">Status Endpoint (JSON)</a></li>
            <li>Use POST {MOVE_PATH} and {TASK_PATH} (see API)</li>
        </ul>
    </body>
    </html>
    """, content_type='text/html')

# ========== App Launch ==========
app = web.Application()
app.add_routes(routes)

if __name__ == "__main__":
    web.run_app(app, host=SERVER_HOST, port=SERVER_PORT)