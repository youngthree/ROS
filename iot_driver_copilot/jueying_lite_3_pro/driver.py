import os
import io
import json
import asyncio
import aiohttp
import aiohttp.web
import cv2
import numpy as np

# --- Environment Variables ---
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", "8080"))
RTSP_PORT = int(os.environ.get("RTSP_PORT", "554"))
RTSP_PATH = os.environ.get("RTSP_PATH", "live")
RTSP_USER = os.environ.get("RTSP_USER", "")
RTSP_PASSWORD = os.environ.get("RTSP_PASSWORD", "")
ROS_API_URL = os.environ.get("ROS_API_URL", f"http://{DEVICE_IP}:5000") # For ROS bridge/RESTful interface, if any.
# --------------------------------------------------

# Helper: Compose RTSP URL (no output/expose, internal use only)
def get_rtsp_url():
    auth = ""
    if RTSP_USER and RTSP_PASSWORD:
        auth = f"{RTSP_USER}:{RTSP_PASSWORD}@"
    return f"rtsp://{auth}{DEVICE_IP}:{RTSP_PORT}/{RTSP_PATH}"

# MJPEG Streaming Handler using OpenCV
async def mjpeg_stream(request):
    boundary = "frame"
    headers = {
        "Content-Type": f"multipart/x-mixed-replace; boundary={boundary}"
    }

    async def stream_response(resp):
        rtsp_url = get_rtsp_url()
        cap = cv2.VideoCapture(rtsp_url)
        if not cap.isOpened():
            await resp.write(
                f"--{boundary}\r\nContent-Type: text/plain\r\n\r\nFailed to open RTSP stream\r\n".encode("utf-8")
            )
            await resp.write_eof()
            return
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    await asyncio.sleep(0.1)
                    continue
                _, jpeg = cv2.imencode('.jpg', frame)
                img_bytes = jpeg.tobytes()
                await resp.write(
                    f"--{boundary}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(img_bytes)}\r\n\r\n".encode("utf-8") + img_bytes + b"\r\n"
                )
                await asyncio.sleep(0.04)  # ~25 FPS
        finally:
            cap.release()
            await resp.write_eof()

    resp = aiohttp.web.StreamResponse(status=200, reason='OK', headers=headers)
    await resp.prepare(request)
    await stream_response(resp)
    return resp

# API: POST /task
async def handle_task(request):
    try:
        payload = await request.json()
        # Example: {"action": "start", "script": "slam"}
        # Here, you should call the actual ROS service or script (not via subprocess), e.g., via HTTP API or Python ROS API.
        # For demonstration, we'll just echo back the command.
        # In production, adapt this to call your ROS Python API or a RESTful service available on the robot.
        # Example for RESTful:
        # async with aiohttp.ClientSession() as session:
        #     async with session.post(f"{ROS_API_URL}/task", json=payload) as resp:
        #         result = await resp.json()
        #         return aiohttp.web.json_response(result)
        # For now, mock:
        return aiohttp.web.json_response({
            "status": "accepted",
            "requested_action": payload.get("action"),
            "requested_script": payload.get("script")
        })
    except Exception as ex:
        return aiohttp.web.json_response({"error": str(ex)}, status=400)

# API: GET /status
async def handle_status(request):
    # Example: Fetch data from ROS/RESTful API or return mock data
    # In production, fetch real-time data
    # async with aiohttp.ClientSession() as session:
    #     async with session.get(f"{ROS_API_URL}/status") as resp:
    #         data = await resp.json()
    #         return aiohttp.web.json_response(data)
    # For now, mock:
    data = {
        "localization": {"x": 1.2, "y": 0.8, "theta": 0.25},
        "navigation": {"status": "idle", "target": None},
        "sensors": {
            "imu": {"yaw": 0.23, "pitch": -0.01, "roll": 0.04},
            "ultrasound_distance": [1.1, 1.2, 1.0, 0.9],
        }
    }
    return aiohttp.web.json_response(data)

# API: POST /move
async def handle_move(request):
    try:
        payload = await request.json()
        # Example: {"linear": {"x": 0.1, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": 0.1}}
        # In production, publish to ROS topic or send to robot via HTTP API.
        # async with aiohttp.ClientSession() as session:
        #     async with session.post(f"{ROS_API_URL}/move", json=payload) as resp:
        #         result = await resp.json()
        #         return aiohttp.web.json_response(result)
        # For now, mock:
        return aiohttp.web.json_response({
            "status": "commanded",
            "velocity_command": payload
        })
    except Exception as ex:
        return aiohttp.web.json_response({"error": str(ex)}, status=400)

# App Setup
app = aiohttp.web.Application()
app.add_routes([
    aiohttp.web.get('/video', mjpeg_stream),
    aiohttp.web.get('/status', handle_status),
    aiohttp.web.post('/task', handle_task),
    aiohttp.web.post('/move', handle_move)
])

if __name__ == "__main__":
    aiohttp.web.run_app(app, host=SERVER_HOST, port=SERVER_PORT)