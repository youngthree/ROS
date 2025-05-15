import os
import asyncio
import json
import aiohttp
import aiohttp.web
import base64

from aiohttp import web

# Environment Variables
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
ROS_API_URL = os.environ.get("ROS_API_URL", f"http://{DEVICE_IP}:8080")  # Example default
RTSP_URL = os.environ.get("RTSP_URL", f"rtsp://{DEVICE_IP}/live")
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", 8000))
RTSP_HTTP_PORT = int(os.environ.get("RTSP_HTTP_PORT", 8081))  # For video proxy

# MJPEG streaming from RTSP
import cv2
import numpy as np

async def mjpeg_stream_handler(request):
    # Connect to RTSP stream using OpenCV
    cap = cv2.VideoCapture(RTSP_URL)
    if not cap.isOpened():
        return web.Response(status=500, text="Unable to connect to RTSP stream")
    response = web.StreamResponse(
        status=200,
        reason='OK',
        headers={
            'Content-Type': 'multipart/x-mixed-replace; boundary=frame'
        }
    )
    await response.prepare(request)
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            _, jpeg = cv2.imencode('.jpg', frame)
            img_bytes = jpeg.tobytes()
            await response.write(b'--frame\r\n')
            await response.write(b'Content-Type: image/jpeg\r\n\r\n')
            await response.write(img_bytes)
            await response.write(b'\r\n')
            await asyncio.sleep(0.04)  # ~25 FPS
    finally:
        cap.release()
    return response

# ROS/REST API Proxy
async def handle_status(request):
    # Fetch status data (localization, navigation, etc.)
    try:
        async with aiohttp.ClientSession() as session:
            async with session.get(f"{ROS_API_URL}/status") as resp:
                if resp.status != 200:
                    return web.Response(status=resp.status, text=await resp.text())
                data = await resp.json()
                return web.json_response(data)
    except Exception as e:
        return web.Response(status=500, text=str(e))

async def handle_move(request):
    # Accept movement commands as JSON and forward to robot
    try:
        payload = await request.json()
        async with aiohttp.ClientSession() as session:
            async with session.post(f"{ROS_API_URL}/move", json=payload) as resp:
                if resp.status != 200:
                    return web.Response(status=resp.status, text=await resp.text())
                data = await resp.json()
                return web.json_response(data)
    except Exception as e:
        return web.Response(status=500, text=str(e))

async def handle_task(request):
    # Start/stop SLAM, LiDAR, Navigation
    try:
        payload = await request.json()
        async with aiohttp.ClientSession() as session:
            async with session.post(f"{ROS_API_URL}/task", json=payload) as resp:
                if resp.status != 200:
                    return web.Response(status=resp.status, text=await resp.text())
                data = await resp.json()
                return web.json_response(data)
    except Exception as e:
        return web.Response(status=500, text=str(e))

def create_main_app():
    app = web.Application()
    app.add_routes([
        web.get('/status', handle_status),
        web.post('/move', handle_move),
        web.post('/task', handle_task),
        web.get('/video', mjpeg_stream_handler)
    ])
    return app

if __name__ == "__main__":
    import threading

    def start_rtsp_server():
        app = web.Application()
        app.router.add_get('/video', mjpeg_stream_handler)
        web.run_app(app, host=SERVER_HOST, port=RTSP_HTTP_PORT)

    # Start RTSP->HTTP MJPEG server in a thread (if different port)
    if RTSP_HTTP_PORT != SERVER_PORT:
        t = threading.Thread(target=start_rtsp_server, daemon=True)
        t.start()

    # Main API server
    app = create_main_app()
    web.run_app(app, host=SERVER_HOST, port=SERVER_PORT)