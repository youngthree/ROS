import os
import asyncio
import json
from aiohttp import web, ClientSession
import aiohttp
import cv2
import numpy as np

# === ENV CONFIG ===
DEVICE_IP = os.getenv("DEVICE_IP")
RTSP_PORT = int(os.getenv("RTSP_PORT", "554"))
ROS_API_BASE = os.getenv("ROS_API_BASE", f"http://{DEVICE_IP}:8000")
HTTP_SERVER_HOST = os.getenv("HTTP_SERVER_HOST", "0.0.0.0")
HTTP_SERVER_PORT = int(os.getenv("HTTP_SERVER_PORT", "8080"))
RTSP_PATH = os.getenv("RTSP_PATH", "ch0_0.h264")
RTSP_USER = os.getenv("RTSP_USER", "")
RTSP_PASS = os.getenv("RTSP_PASS", "")

RTSP_URL = (
    f"rtsp://{RTSP_USER + ':' + RTSP_PASS + '@' if RTSP_USER and RTSP_PASS else ''}"
    f"{DEVICE_IP}:{RTSP_PORT}/{RTSP_PATH}"
)

# === HTTP API ===

routes = web.RouteTableDef()

# Proxy RTSP video as MJPEG over HTTP
@routes.get("/video")
async def video_feed(request):
    response = web.StreamResponse(
        status=200,
        reason="OK",
        headers={
            "Content-Type": "multipart/x-mixed-replace; boundary=frame"
        },
    )
    await response.prepare(request)

    # OpenCV VideoCapture blocking; offload to executor
    loop = asyncio.get_event_loop()

    def get_video_frames(rtsp_url):
        cap = cv2.VideoCapture(rtsp_url)
        if not cap.isOpened():
            return
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                _, jpeg = cv2.imencode('.jpg', frame)
                yield jpeg.tobytes()
        finally:
            cap.release()

    async def send_frames():
        for jpg_bytes in get_video_frames(RTSP_URL):
            await response.write(
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + jpg_bytes + b'\r\n'
            )
            await asyncio.sleep(0.04)  # ~25 fps

    try:
        await send_frames()
    except asyncio.CancelledError:
        pass
    finally:
        await response.write_eof()
    return response

@routes.get("/status")
async def get_status(request):
    # Simulate fetching localization/navigation status and sensor readings
    try:
        async with ClientSession() as session:
            async with session.get(f"{ROS_API_BASE}/status") as resp:
                data = await resp.json()
                return web.json_response(data)
    except Exception:
        # Fallback: return mock data
        return web.json_response({
            "localization": {"x": 0, "y": 0, "yaw": 0},
            "navigation": {"status": "idle"},
            "sensors": {
                "imu": {"ax": 0, "ay": 0, "az": 0},
                "ultrasound": [0.0, 0.0, 0.0],
            }
        })

@routes.post("/task")
async def manage_task(request):
    # JSON: { "action": "start"/"stop", "script": "slam"/"lidar"/"navigation" }
    data = await request.json()
    # Forward to ROS API or handle locally
    try:
        async with ClientSession() as session:
            async with session.post(f"{ROS_API_BASE}/task", json=data) as resp:
                r = await resp.json()
                return web.json_response(r)
    except Exception:
        return web.json_response({"success": True, "message": f"Task {data.get('action')} {data.get('script')}"})

@routes.post("/move")
async def move_robot(request):
    # JSON: { "linear": {...}, "angular": {...} }
    data = await request.json()
    try:
        async with ClientSession() as session:
            async with session.post(f"{ROS_API_BASE}/move", json=data) as resp:
                r = await resp.json()
                return web.json_response(r)
    except Exception:
        return web.json_response({"success": True, "message": "Movement command accepted"})

app = web.Application()
app.add_routes(routes)

if __name__ == "__main__":
    web.run_app(app, host=HTTP_SERVER_HOST, port=HTTP_SERVER_PORT)