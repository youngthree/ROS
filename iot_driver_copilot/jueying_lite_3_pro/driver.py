import os
import asyncio
import json
import aiohttp
import aiohttp.web
import cv2
import numpy as np

# ENVIRONMENT CONFIGURATION
ROBOT_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
ROS_API_URL = os.environ.get("ROS_API_URL", f"http://{ROBOT_IP}:8000")
RTSP_URL = os.environ.get("RTSP_URL", f"rtsp://{ROBOT_IP}:8554/live")
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", "8080"))
JPEG_QUALITY = int(os.environ.get("JPEG_QUALITY", "80"))  # for video streaming

routes = aiohttp.web.RouteTableDef()

# --------- /task: Manage operational scripts (POST) ----------
@routes.post('/task')
async def manage_task(request):
    data = await request.json()
    action = data.get("action")
    script_type = data.get("script_type")
    if not action or not script_type:
        return aiohttp.web.json_response({"error": "Missing 'action' or 'script_type'"}, status=400)
    # Simulate script management (in real system, trigger ROS service or REST endpoint)
    # Here, we assume an HTTP API for script control is available on the robot
    try:
        async with aiohttp.ClientSession() as session:
            resp = await session.post(
                f"{ROS_API_URL}/scripts/manage", 
                json={"action": action, "type": script_type},
                timeout=5
            )
            result = await resp.json()
            return aiohttp.web.json_response(result, status=resp.status)
    except Exception as e:
        return aiohttp.web.json_response({"error": str(e)}, status=502)

# --------- /status: Get status data (GET) ----------
@routes.get('/status')
async def get_status(request):
    # Simulate fetching sensor/status data from REST API or ROS bridge
    try:
        async with aiohttp.ClientSession() as session:
            resp = await session.get(f"{ROS_API_URL}/status", timeout=5)
            result = await resp.json()
            return aiohttp.web.json_response(result, status=resp.status)
    except Exception as e:
        return aiohttp.web.json_response({"error": str(e)}, status=502)

# --------- /move: Send movement command (POST) ----------
@routes.post('/move')
async def move_robot(request):
    data = await request.json()
    # Expects {"linear": ..., "angular": ...} or compatible
    try:
        async with aiohttp.ClientSession() as session:
            resp = await session.post(
                f"{ROS_API_URL}/cmd_vel",
                json=data,
                timeout=3
            )
            if resp.status == 200:
                return aiohttp.web.json_response({"status": "ok"})
            else:
                return aiohttp.web.json_response({"error": "ROS API error"}, status=resp.status)
    except Exception as e:
        return aiohttp.web.json_response({"error": str(e)}, status=502)

# --------- /video: Proxy RTSP to HTTP MJPEG stream (GET) ----------
@routes.get('/video')
async def mjpeg_video(request):
    async def video_stream(resp):
        cap = cv2.VideoCapture(RTSP_URL)
        if not cap.isOpened():
            await resp.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
            await resp.write(cv2.imencode('.jpg', np.zeros((480,640,3), np.uint8))[1].tobytes())
            await resp.write(b"\r\n")
            return
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    await asyncio.sleep(0.05)
                    continue
                _, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
                await resp.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg.tobytes() + b"\r\n")
                await asyncio.sleep(0.04)  # ~25 FPS
        finally:
            cap.release()

    headers = {
        "Content-Type": "multipart/x-mixed-replace; boundary=frame",
        "Cache-Control": "no-cache",
        "Pragma": "no-cache"
    }
    response = aiohttp.web.StreamResponse(status=200, reason='OK', headers=headers)
    await response.prepare(request)
    await video_stream(response)
    return response

# --------- Server setup ----------
app = aiohttp.web.Application()
app.add_routes(routes)

if __name__ == '__main__':
    aiohttp.web.run_app(app, host=SERVER_HOST, port=SERVER_PORT)