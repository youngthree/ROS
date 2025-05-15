import os
import asyncio
import json
from aiohttp import web
import aiohttp
import cv2
import numpy as np

# Environment config
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", "8080"))
RTSP_PORT = int(os.environ.get("RTSP_PORT", "8554"))  # Only used if RTSP is needed

# Example RTSP URL construction
RTSP_PATH = os.environ.get("RTSP_PATH", "live/ch0")
RTSP_USER = os.environ.get("RTSP_USER")
RTSP_PASS = os.environ.get("RTSP_PASS")
if RTSP_USER and RTSP_PASS:
    RTSP_URL = f"rtsp://{RTSP_USER}:{RTSP_PASS}@{DEVICE_IP}:{RTSP_PORT}/{RTSP_PATH}"
else:
    RTSP_URL = f"rtsp://{DEVICE_IP}:{RTSP_PORT}/{RTSP_PATH}"

# Dummy in-memory state for /status endpoint
robot_status = {
    "localization": {"x": 0.0, "y": 0.0, "theta": 0.0},
    "navigation": {"status": "idle"},
    "sensors": {"imu": {}, "odom": {}, "joint_states": {}}
}

# Dummy operational script management
operational_scripts = {
    "slam": False,
    "lidar": False,
    "navigation": False
}

routes = web.RouteTableDef()

@routes.post('/task')
async def manage_task(request):
    req = await request.json()
    action = req.get("action")
    script_type = req.get("script_type")
    if script_type not in operational_scripts:
        return web.json_response({"result": "error", "reason": "unknown script_type"}, status=400)
    if action == "start":
        operational_scripts[script_type] = True
        return web.json_response({"result": "ok", "started": script_type})
    elif action == "stop":
        operational_scripts[script_type] = False
        return web.json_response({"result": "ok", "stopped": script_type})
    else:
        return web.json_response({"result": "error", "reason": "unknown action"}, status=400)

@routes.get('/status')
async def get_status(request):
    # Example: This should be replaced by actual data fetch from ROS or UDP, etc.
    return web.json_response(robot_status)

@routes.post('/move')
async def move_robot(request):
    req = await request.json()
    # Example: In real driver, publish to ROS topic or send UDP packet
    # Here just echo back for demo
    robot_status["last_cmd_vel"] = req
    return web.json_response({"result": "ok", "cmd": req})

@routes.get('/video')
async def video_stream(request):
    """
    Streams RTSP video as HTTP multipart stream (MJPEG).
    """
    async def mjpeg_response(resp):
        # OpenCV VideoCapture from RTSP
        cap = cv2.VideoCapture(RTSP_URL)
        if not cap.isOpened():
            await resp.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
            await resp.write(b"Camera unavailable")
            return
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                # Encode frame as JPEG
                ret, jpeg = cv2.imencode('.jpg', frame)
                if not ret:
                    continue
                img_bytes = jpeg.tobytes()
                part = (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + img_bytes + b"\r\n"
                )
                await resp.write(part)
                await asyncio.sleep(0.04)  # ~25fps
        finally:
            cap.release()

    resp = web.StreamResponse(
        status=200,
        reason='OK',
        headers={
            'Content-Type': 'multipart/x-mixed-replace; boundary=frame'
        }
    )
    await resp.prepare(request)
    await mjpeg_response(resp)
    return resp

app = web.Application()
app.add_routes(routes)

if __name__ == '__main__':
    web.run_app(app, host=SERVER_HOST, port=SERVER_PORT)