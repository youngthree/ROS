import os
import asyncio
import json
import aiohttp
import aiohttp.web
import base64

import cv2
import numpy as np

# Environment variables
DEVICE_IP = os.environ.get('DEVICE_IP', '127.0.0.1')
SERVER_HOST = os.environ.get('SERVER_HOST', '0.0.0.0')
SERVER_PORT = int(os.environ.get('SERVER_PORT', 8080))
RTSP_PORT = int(os.environ.get('RTSP_PORT', 8554))
RTSP_PATH = os.environ.get('RTSP_PATH', 'live/stream')
RTSP_USERNAME = os.environ.get('RTSP_USERNAME', '')
RTSP_PASSWORD = os.environ.get('RTSP_PASSWORD', '')
ROSBRIDGE_WS_URL = os.environ.get('ROSBRIDGE_WS_URL', f'ws://{DEVICE_IP}:9090')

# RTSP URL construction
def get_rtsp_url():
    if RTSP_USERNAME and RTSP_PASSWORD:
        return f"rtsp://{RTSP_USERNAME}:{RTSP_PASSWORD}@{DEVICE_IP}:{RTSP_PORT}/{RTSP_PATH}"
    else:
        return f"rtsp://{DEVICE_IP}:{RTSP_PORT}/{RTSP_PATH}"

# ---- HTTP Endpoints ----

routes = aiohttp.web.RouteTableDef()

# POST /task - Manage operational scripts
@routes.post('/task')
async def handle_task(request):
    try:
        data = await request.json()
        action = data.get('action')
        script_type = data.get('type')
        # Simulate management (normally would interact with ROS or device API)
        # Here, placeholder response, no external command exec
        # An actual implementation would publish to ROS or call device HTTP API
        return aiohttp.web.json_response({
            "status": "ok",
            "action": action,
            "type": script_type
        })
    except Exception as e:
        return aiohttp.web.json_response({"error": str(e)}, status=400)

# GET /status - Fetch status data
@routes.get('/status')
async def handle_status(request):
    # Placeholder: Simulate status data fetch from ROS or device API
    # In a real scenario, you would fetch via ROS client or HTTP API
    status = {
        "localization": {
            "x": 1.23,
            "y": 4.56,
            "theta": 0.78
        },
        "navigation_status": "IDLE",
        "imu": {
            "accel_x": 0.01,
            "accel_y": 0.02,
            "accel_z": 9.81
        },
        "battery": {
            "voltage": 25.5,
            "percentage": 87
        }
    }
    return aiohttp.web.json_response(status)

# POST /move - Send movement commands
@routes.post('/move')
async def handle_move(request):
    try:
        data = await request.json()
        # Normally, would publish this to a ROS topic, here just echo back
        # For actual device, integrate with ROSBridge or device API
        return aiohttp.web.json_response({
            "status": "command_received",
            "cmd": data
        })
    except Exception as e:
        return aiohttp.web.json_response({"error": str(e)}, status=400)

# MJPEG streaming over HTTP from RTSP
@routes.get('/video')
async def video_stream(request):
    boundary = "frame"
    response = aiohttp.web.StreamResponse(
        status=200,
        reason='OK',
        headers={
            'Content-Type': f'multipart/x-mixed-replace; boundary=--{boundary}',
            'Cache-Control': 'no-cache',
        }
    )
    await response.prepare(request)

    rtsp_url = get_rtsp_url()
    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        await response.write(b"--frame\r\nContent-Type: text/plain\r\n\r\nCould not open video stream\r\n\r\n")
        await response.write_eof()
        return response

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                await asyncio.sleep(0.05)
                continue
            _, jpeg = cv2.imencode('.jpg', frame)
            data = jpeg.tobytes()
            await response.write(
                b"--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % (
                    boundary.encode(), len(data)
                ) + data + b"\r\n"
            )
            await response.drain()
            await asyncio.sleep(0.05)  # ~20 FPS
    except asyncio.CancelledError:
        pass
    finally:
        cap.release()
        await response.write_eof()
    return response

# Root page with simple HTML player for testing
@routes.get('/')
async def index(request):
    html = """
    <html>
    <head><title>Jueying Lite3 Pro Driver</title></head>
    <body>
    <h2>Status <a href="/status">(JSON)</a></h2>
    <h2>Video Stream:</h2>
    <img src="/video" style="max-width:90vw;" />
    </body>
    </html>
    """
    return aiohttp.web.Response(text=html, content_type='text/html')

# ---- Application Factory ----

def main():
    app = aiohttp.web.Application()
    app.add_routes(routes)
    aiohttp.web.run_app(app, host=SERVER_HOST, port=SERVER_PORT)

if __name__ == "__main__":
    main()