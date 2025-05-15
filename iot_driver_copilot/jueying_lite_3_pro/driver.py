import os
import asyncio
import json
import aiohttp
import aiohttp.web
import base64
import cv2
import numpy as np

# Environment Variable Configuration
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", "8080"))
RTSP_PORT = int(os.environ.get("RTSP_PORT", "554"))

# RTSP Stream URL
RTSP_URL = os.environ.get("RTSP_URL", f"rtsp://{DEVICE_IP}:{RTSP_PORT}/live")

# ROS/UDP endpoints/config (stub, for demonstration)
ROS_API_URL = os.environ.get("ROS_API_URL", f"http://{DEVICE_IP}:5000")
UDP_STATUS_PORT = int(os.environ.get("UDP_STATUS_PORT", "15001"))

# MJPEG boundary
MJPEG_BOUNDARY = b"--frame\r\n"

routes = aiohttp.web.RouteTableDef()

# /task: Manage operational scripts (stub: simulates start/stop of scripts)
@routes.post("/task")
async def task_handler(request):
    body = await request.json()
    action = body.get("action")
    script_type = body.get("script_type")
    # Simulate script management (can be replaced with ROS/SSH calls)
    result = {
        "action": action,
        "script_type": script_type,
        "status": "success"
    }
    return aiohttp.web.json_response(result)

# /status: Returns localization, navigation, IMU, etc. (stub: simulates data)
@routes.get("/status")
async def status_handler(request):
    # Simulate fetching status data (replace with UDP/ROS/REST as needed)
    status = {
        "localization": {
            "x": 1.2, "y": 3.4, "theta": 0.12
        },
        "navigation": {
            "status": "idle",
            "goal": None
        },
        "imu": {
            "linear_acceleration": [0.0, 0.0, 9.8],
            "angular_velocity": [0.01, 0.01, 0.0],
            "orientation": [0.0, 0.0, 0.0, 1.0]
        },
        "battery": {
            "percent": 87
        }
    }
    return aiohttp.web.json_response(status)

# /move: Receives velocity commands (stub: simulates command send)
@routes.post("/move")
async def move_handler(request):
    body = await request.json()
    # Here you would send this as a ROS topic or UDP packet
    result = {
        "received": body,
        "status": "sent"
    }
    return aiohttp.web.json_response(result)

# /video: Proxies RTSP video to HTTP MJPEG stream for browser
@routes.get("/video")
async def video_stream_handler(request):
    async def mjpeg_stream(resp):
        cap = cv2.VideoCapture(RTSP_URL)
        if not cap.isOpened():
            await resp.write(b"HTTP/1.1 503 Service Unavailable\r\n\r\n")
            return
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                _, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                img_bytes = jpeg.tobytes()
                await resp.write(
                    MJPEG_BOUNDARY +
                    b"Content-Type: image/jpeg\r\n" +
                    f"Content-Length: {len(img_bytes)}\r\n\r\n".encode() +
                    img_bytes + b"\r\n"
                )
                await asyncio.sleep(0.04)  # ~25 FPS
        finally:
            cap.release()
    headers = {
        "Content-Type": "multipart/x-mixed-replace; boundary=frame"
    }
    resp = aiohttp.web.StreamResponse(status=200, reason='OK', headers=headers)
    await resp.prepare(request)
    await mjpeg_stream(resp)
    return resp

app = aiohttp.web.Application()
app.add_routes(routes)

if __name__ == "__main__":
    aiohttp.web.run_app(app, host=SERVER_HOST, port=SERVER_PORT)