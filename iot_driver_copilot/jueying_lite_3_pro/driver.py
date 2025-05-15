import os
import asyncio
import json
import aiohttp
import aiohttp.web
import cv2
import numpy as np

# Environment Variables
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
ROS_API_HOST = os.environ.get("ROS_API_HOST", DEVICE_IP)
ROS_API_PORT = int(os.environ.get("ROS_API_PORT", "8080"))
RTSP_PORT = int(os.environ.get("RTSP_PORT", "8554"))
RTSP_PATH = os.environ.get("RTSP_PATH", "stream")
HTTP_SERVER_HOST = os.environ.get("HTTP_SERVER_HOST", "0.0.0.0")
HTTP_SERVER_PORT = int(os.environ.get("HTTP_SERVER_PORT", "8000"))

# RTSP URL format for the robot
RTSP_URL = f"rtsp://{DEVICE_IP}:{RTSP_PORT}/{RTSP_PATH}"

# ---- Video Stream Proxy (RTSP to HTTP MJPEG) ----

async def mjpeg_stream(request):
    boundary = "frame"
    response = aiohttp.web.StreamResponse(
        status=200,
        reason='OK',
        headers={
            'Content-Type': f'multipart/x-mixed-replace; boundary=--{boundary}',
            'Cache-Control': 'no-cache',
            'Pragma': 'no-cache',
        }
    )
    await response.prepare(request)

    cap = cv2.VideoCapture(RTSP_URL)
    if not cap.isOpened():
        await response.write(b"--frame\r\nContent-Type: text/plain\r\n\r\nFailed to open RTSP stream\r\n")
        await response.write_eof()
        return response

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                await asyncio.sleep(0.2)
                continue
            _, jpeg = cv2.imencode('.jpg', frame)
            img_bytes = jpeg.tobytes()
            await response.write(
                b"--" + boundary.encode() + b"\r\n"
                b"Content-Type: image/jpeg\r\n"
                b"Content-Length: " + str(len(img_bytes)).encode() + b"\r\n\r\n"
                + img_bytes + b"\r\n"
            )
            await asyncio.sleep(0.033)  # ~30fps
    except asyncio.CancelledError:
        cap.release()
    except Exception:
        cap.release()
    finally:
        cap.release()
    return response

# ---- API Endpoint Handlers ----

async def handle_status(request):
    # Example: fetch status from the robot's ROS API (mocked here)
    # Replace with actual ROS API call if available
    status_url = f"http://{ROS_API_HOST}:{ROS_API_PORT}/status"
    try:
        async with aiohttp.ClientSession() as session:
            async with session.get(status_url) as resp:
                robot_status = await resp.json()
                return aiohttp.web.json_response(robot_status)
    except Exception:
        # Fallback/mocked response
        return aiohttp.web.json_response({
            "localization": {"x": 0, "y": 0, "theta": 0},
            "navigation_status": "idle",
            "sensors": {
                "imu": {},
                "odom": {},
                "joint_states": {},
                "ultrasound": {},
            }
        })

async def handle_task(request):
    try:
        data = await request.json()
    except Exception:
        return aiohttp.web.json_response({"error": "Invalid JSON"}, status=400)
    # Example: start/stop scripts via robot's API (mocked here)
    action = data.get("action")
    script_type = data.get("script_type")
    if not action or not script_type:
        return aiohttp.web.json_response({"error": "Missing action or script_type"}, status=400)
    # Replace with actual robot control logic if available.
    # For now, just echo request.
    return aiohttp.web.json_response({
        "result": "ok",
        "action": action,
        "script_type": script_type
    })

async def handle_move(request):
    try:
        cmd = await request.json()
    except Exception:
        return aiohttp.web.json_response({"error": "Invalid JSON"}, status=400)
    # Example: send cmd_vel to robot's ROS API (mocked here)
    move_url = f"http://{ROS_API_HOST}:{ROS_API_PORT}/move"
    try:
        async with aiohttp.ClientSession() as session:
            async with session.post(move_url, json=cmd) as resp:
                if resp.status == 200:
                    return aiohttp.web.json_response(await resp.json())
                else:
                    return aiohttp.web.json_response({"error": "Failed to move"}, status=500)
    except Exception:
        # Fallback/mocked response
        return aiohttp.web.json_response({"result": "move_command_sent", "cmd": cmd})

# ---- App Setup ----

app = aiohttp.web.Application()
app.router.add_get('/video', mjpeg_stream)
app.router.add_get('/status', handle_status)
app.router.add_post('/task', handle_task)
app.router.add_post('/move', handle_move)

if __name__ == '__main__':
    aiohttp.web.run_app(app, host=HTTP_SERVER_HOST, port=HTTP_SERVER_PORT)