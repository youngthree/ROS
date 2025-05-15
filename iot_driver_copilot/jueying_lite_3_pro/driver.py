import os
import asyncio
import json
import aiohttp
import aiohttp.web
import cv2
import numpy as np

# Env vars
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
ROS_API_HOST = os.environ.get("ROS_API_HOST", "127.0.0.1")
ROS_API_PORT = int(os.environ.get("ROS_API_PORT", 5000))
RTSP_URL = os.environ.get("RTSP_URL", f"rtsp://{DEVICE_IP}/camera")
HTTP_SERVER_HOST = os.environ.get("HTTP_SERVER_HOST", "0.0.0.0")
HTTP_SERVER_PORT = int(os.environ.get("HTTP_SERVER_PORT", 8080))
# Only configure ports/protocols actually used

routes = aiohttp.web.RouteTableDef()

# --- Helper for RTSP to HTTP MJPEG conversion ---
async def mjpeg_stream_response(rtsp_url):
    # Use OpenCV to capture RTSP stream and yield JPEG frames over HTTP
    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        raise aiohttp.web.HTTPInternalServerError(reason="Could not open RTSP stream.")
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                await asyncio.sleep(0.05)
                continue
            ret, jpeg = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            jpg_bytes = jpeg.tobytes()
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" +
                   jpg_bytes + b"\r\n")
            await asyncio.sleep(0.04)  # ~25 FPS
    finally:
        cap.release()

@routes.get('/video')
async def video_feed(request):
    response = aiohttp.web.StreamResponse(
        status=200,
        reason='OK',
        headers={
            'Content-Type': 'multipart/x-mixed-replace; boundary=frame'
        }
    )
    await response.prepare(request)
    async for chunk in mjpeg_stream_response(RTSP_URL):
        await response.write(chunk)
    return response

# --- API: POST /task ---
@routes.post('/task')
async def task_handler(request):
    """
    Manage operational scripts such as SLAM, LiDAR, or navigation.
    Payload: {"action": "start"/"stop", "script_type": "slam"/"lidar"/"navigation"}
    """
    try:
        data = await request.json()
        action = data.get("action")
        script_type = data.get("script_type")
        if action not in ["start", "stop"] or script_type not in ["slam", "lidar", "navigation"]:
            return aiohttp.web.json_response({"error": "Invalid action or script_type"}, status=400)
        # Forward command to ROS API or device
        async with aiohttp.ClientSession() as session:
            url = f"http://{ROS_API_HOST}:{ROS_API_PORT}/driver/script"
            payload = {"action": action, "script_type": script_type}
            async with session.post(url, json=payload) as resp:
                resp_data = await resp.json()
        return aiohttp.web.json_response(resp_data)
    except Exception as e:
        return aiohttp.web.json_response({"error": str(e)}, status=500)

# --- API: GET /status ---
@routes.get('/status')
async def status_handler(request):
    """
    Fetch current status data including localization and navigation metrics.
    """
    try:
        async with aiohttp.ClientSession() as session:
            url = f"http://{ROS_API_HOST}:{ROS_API_PORT}/driver/status"
            async with session.get(url) as resp:
                data = await resp.json()
        return aiohttp.web.json_response(data)
    except Exception as e:
        return aiohttp.web.json_response({"error": str(e)}, status=500)

# --- API: POST /move ---
@routes.post('/move')
async def move_handler(request):
    """
    Send movement commands to the robot.
    Payload: {"linear": {"x": float, "y": float, "z": float}, "angular": {"x": float, "y": float, "z": float}}
    """
    try:
        data = await request.json()
        async with aiohttp.ClientSession() as session:
            url = f"http://{ROS_API_HOST}:{ROS_API_PORT}/driver/move"
            async with session.post(url, json=data) as resp:
                resp_data = await resp.json()
        return aiohttp.web.json_response(resp_data)
    except Exception as e:
        return aiohttp.web.json_response({"error": str(e)}, status=500)

# --- App startup ---
app = aiohttp.web.Application()
app.add_routes(routes)

if __name__ == "__main__":
    aiohttp.web.run_app(app, host=HTTP_SERVER_HOST, port=HTTP_SERVER_PORT)