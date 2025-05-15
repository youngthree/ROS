import os
import asyncio
import json
import aiohttp
import aiohttp.web
import websockets
import cv2
import numpy as np

# --- ENVIRONMENT VARIABLES ---
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
ROS_API_HOST = os.environ.get("ROS_API_HOST", DEVICE_IP)
ROS_API_PORT = int(os.environ.get("ROS_API_PORT", "9090"))
RTSP_URL = os.environ.get("RTSP_URL", f"rtsp://{DEVICE_IP}/live")
HTTP_SERVER_HOST = os.environ.get("HTTP_SERVER_HOST", "0.0.0.0")
HTTP_SERVER_PORT = int(os.environ.get("HTTP_SERVER_PORT", "8000"))

# --- ROS HTTP API ENDPOINTS (assumed for demonstration) ---
ROS_CMD_VEL_TOPIC = os.environ.get("ROS_CMD_VEL_TOPIC", "/cmd_vel")
ROS_NAV_STATUS_TOPIC = os.environ.get("ROS_NAV_STATUS_TOPIC", "/navigation/status")
ROS_LOCALIZATION_TOPIC = os.environ.get("ROS_LOCALIZATION_TOPIC", "/localization/status")
ROS_SCRIPT_API_URL = os.environ.get("ROS_SCRIPT_API_URL", f"http://{ROS_API_HOST}:{ROS_API_PORT}/scripts")

# --- HTTP SERVER ---

routes = aiohttp.web.RouteTableDef()

# -- UTILS --

async def fetch_navigation_status():
    # Placeholder: Fetch navigation/localization/status from ROS API or UDP endpoint
    # Here we simulate with dummy data for demonstration
    status = {
        "localization": {"x": 1.23, "y": 4.56, "theta": 0.78},
        "navigation": {"active": True, "goal": [2.0, 3.0]},
        "sensors": {
            "imu": {"linear_acceleration": [0.01, 0.0, 9.8]},
            "ultrasound_distance": [1.2, 2.5, 0.8]
        }
    }
    return status

async def ros_publish_cmd_vel(cmd):
    async with aiohttp.ClientSession() as session:
        url = f"http://{ROS_API_HOST}:{ROS_API_PORT}/rosapi/publish"
        payload = {
            "topic": ROS_CMD_VEL_TOPIC,
            "msg": cmd
        }
        async with session.post(url, json=payload) as resp:
            return await resp.text()

async def manage_script(action, script_type):
    async with aiohttp.ClientSession() as session:
        url = f"{ROS_SCRIPT_API_URL}/{script_type}/{action}"
        async with session.post(url) as resp:
            return await resp.text()

# --- API ENDPOINTS ---

@routes.post('/move')
async def move(request):
    try:
        data = await request.json()
        # Forward cmd_vel to ROS (assume JSON matches ROS message for geometry_msgs/Twist)
        result = await ros_publish_cmd_vel(data)
        return aiohttp.web.json_response({"result": result})
    except Exception as e:
        return aiohttp.web.json_response({"error": str(e)}, status=400)

@routes.get('/status')
async def status(request):
    # Fetch status data from ROS/UDP/etc.
    status = await fetch_navigation_status()
    return aiohttp.web.json_response(status)

@routes.post('/task')
async def task(request):
    try:
        data = await request.json()
        action = data.get("action", "start")
        script_type = data.get("script_type", "slam")
        result = await manage_script(action, script_type)
        return aiohttp.web.json_response({"result": result})
    except Exception as e:
        return aiohttp.web.json_response({"error": str(e)}, status=400)

# --- MJPEG VIDEO STREAM (RTSP -> HTTP) ---

async def mjpeg_video_stream(request):
    # Use OpenCV to connect to RTSP and yield as HTTP multipart
    boundary = "frame"
    response = aiohttp.web.StreamResponse(
        status=200,
        reason='OK',
        headers={
            'Content-Type': f'multipart/x-mixed-replace; boundary={boundary}',
            'Cache-Control': 'no-cache',
            'Connection': 'close',
            'Pragma': 'no-cache',
        }
    )
    await response.prepare(request)
    cap = cv2.VideoCapture(RTSP_URL)
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                await asyncio.sleep(0.1)
                continue
            ret, jpeg = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            await response.write(
                b"--" + boundary.encode() + b"\r\n"
                + b"Content-Type: image/jpeg\r\n"
                + f"Content-Length: {len(jpeg)}\r\n\r\n".encode()
                + jpeg.tobytes() + b"\r\n"
            )
            await asyncio.sleep(0.033)  # ~30fps
    except asyncio.CancelledError:
        pass
    finally:
        cap.release()
        await response.write_eof()
    return response

routes.get('/video')(mjpeg_video_stream)

# --- MAIN ---

app = aiohttp.web.Application()
app.add_routes(routes)

if __name__ == "__main__":
    aiohttp.web.run_app(app, host=HTTP_SERVER_HOST, port=HTTP_SERVER_PORT)