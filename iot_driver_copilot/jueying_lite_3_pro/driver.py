import os
import asyncio
import json
import aiohttp
import aiohttp.web
import websockets
import cv2
import numpy as np

# Environment Variables
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
ROS_API_URL = os.environ.get("ROS_API_URL", f"http://{DEVICE_IP}:9090")
RTSP_URL = os.environ.get("RTSP_URL", f"rtsp://{DEVICE_IP}/live")
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", "8080"))
HTTP_VIDEO_PATH = os.environ.get("HTTP_VIDEO_PATH", "/video")
HTTP_VIDEO_WS_PATH = os.environ.get("HTTP_VIDEO_WS_PATH", "/video/ws")

# Helper function: get robot status (simulate ROS/REST API or UDP)
async def get_status():
    # Simulate data, replace with real ROS/REST/UDP retrieval if available
    return {
        "localization": {"x": 1.23, "y": 4.56, "theta": 0.78},
        "navigation_status": "IDLE",
        "imu": {"ax": 0.01, "ay": 0.02, "az": 9.81},
        "ultrasound_distance": [0.23, 0.21, 0.19, 0.22],
        "joint_states": {"joint1": 0.1, "joint2": -0.2},
        "handle_state": "NEUTRAL"
    }

# Helper function: send movement command (simulate ROS/REST API or UDP)
async def send_move_command(cmd):
    # Here, send to ROS or UDP as per device's API
    # Simulate success
    return {"result": "ok", "sent": cmd}

# Helper function: manage tasks (simulate ROS/REST API or SSH)
async def manage_task(action, script_type):
    # Here, start/stop SLAM/LiDAR/Navigation etc. via API
    # Simulate success
    return {"result": "ok", "action": action, "script_type": script_type}

# HTTP Handlers
async def handle_status(request):
    status = await get_status()
    return aiohttp.web.json_response(status)

async def handle_move(request):
    data = await request.json()
    resp = await send_move_command(data)
    return aiohttp.web.json_response(resp)

async def handle_task(request):
    data = await request.json()
    action = data.get("action")
    script_type = data.get("script_type")
    resp = await manage_task(action, script_type)
    return aiohttp.web.json_response(resp)

# Video Streaming to HTTP MJPEG
async def mjpeg_stream(request):
    # Connect to RTSP and transcode to MJPEG on the fly
    cap = cv2.VideoCapture(RTSP_URL)
    if not cap.isOpened():
        return aiohttp.web.Response(status=503, text="Unable to open RTSP stream.")
    response = aiohttp.web.StreamResponse(
        status=200,
        reason='OK',
        headers={
            'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
            'Cache-Control': 'no-cache',
            'Connection': 'close'
        }
    )
    await response.prepare(request)
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            ret, jpg = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            await response.write(b'--frame\r\n')
            await response.write(b'Content-Type: image/jpeg\r\n\r\n')
            await response.write(jpg.tobytes())
            await response.write(b'\r\n')
            await asyncio.sleep(0.04)  # ~25fps
    except asyncio.CancelledError:
        pass
    finally:
        cap.release()
    await response.write_eof()
    return response

# Video streaming to WebSocket (JPEG frames for browser)
async def ws_video_handler(request):
    ws = aiohttp.web.WebSocketResponse()
    await ws.prepare(request)
    cap = cv2.VideoCapture(RTSP_URL)
    if not cap.isOpened():
        await ws.send_json({"error": "Unable to open RTSP stream."})
        await ws.close()
        return ws
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            ret, jpg = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            await ws.send_bytes(jpg.tobytes())
            await asyncio.sleep(0.04)
    except asyncio.CancelledError:
        pass
    finally:
        cap.release()
        await ws.close()
    return ws

# Application Factory
def create_app():
    app = aiohttp.web.Application()
    app.router.add_get('/status', handle_status)
    app.router.add_post('/move', handle_move)
    app.router.add_post('/task', handle_task)
    app.router.add_get(HTTP_VIDEO_PATH, mjpeg_stream)
    app.router.add_get(HTTP_VIDEO_WS_PATH, ws_video_handler)
    return app

if __name__ == '__main__':
    app = create_app()
    aiohttp.web.run_app(app, host=SERVER_HOST, port=SERVER_PORT)