import os
import io
import json
import asyncio
from typing import Optional
from fastapi import FastAPI, Request, Response, status
from fastapi.responses import StreamingResponse, JSONResponse
from pydantic import BaseModel
import aiohttp

# ============ Environment Variables ============
ROBOT_IP = os.environ.get('DEVICE_IP', '127.0.0.1')
ROS_API_PORT = int(os.environ.get('ROS_API_PORT', '9090'))  # For ROSBridge websocket API
UDP_PORT = int(os.environ.get('UDP_PORT', '15000'))         # For UDP status data (simulate if not used)
RTSP_URL = os.environ.get('RTSP_URL', f'rtsp://{ROBOT_IP}/video')  # RTSP stream URL
HTTP_SERVER_HOST = os.environ.get('HTTP_SERVER_HOST', '0.0.0.0')
HTTP_SERVER_PORT = int(os.environ.get('HTTP_SERVER_PORT', '8000'))

# ============ FastAPI App ============
app = FastAPI()

# ============ Models ============
class TaskRequest(BaseModel):
    action: str  # e.g., "start" or "stop"
    script: str  # e.g., "SLAM", "LiDAR", "navigation"

class MoveRequest(BaseModel):
    linear_x: float
    linear_y: Optional[float] = 0.0
    linear_z: Optional[float] = 0.0
    angular_x: Optional[float] = 0.0
    angular_y: Optional[float] = 0.0
    angular_z: float

# ============ ROSBridge Util (WebSocket for ROS API) ============
async def ros_send_cmd_vel(cmd: dict):
    """
    Send /cmd_vel via ROSBridge websocket.
    """
    import websockets
    uri = f"ws://{ROBOT_IP}:{ROS_API_PORT}"
    ros_message = {
        "op": "publish",
        "topic": "/cmd_vel",
        "msg": cmd
    }
    async with websockets.connect(uri) as ws:
        await ws.send(json.dumps(ros_message))
        # Optionally, receive response (ROSBridge may not respond for 'publish')

# ============ Status Fetcher Example (UDP or ROSBridge) ============
async def fetch_status():
    # Placeholder: In real case, fetch from UDP or ROSBridge topic
    # Here simulate with static data for example purposes
    return {
        "localization": {"x": 1.23, "y": 4.56, "theta": 0.78},
        "navigation": {"status": "idle", "target": None},
        "sensors": {
            "imu": {"accel": [0.01, 0.02, 9.81], "gyro": [0.0, 0.0, 0.001]},
            "ultrasound": [1.2, 1.1, 0.9, 1.3]
        }
    }

# ============ RTSP to HTTP MJPEG Proxy ============
async def rtsp_to_mjpeg_stream(rtsp_url):
    """
    Connect to RTSP, extract MJPEG stream, yield as HTTP multipart stream.
    Only works if RTSP stream is already MJPEG or JPEG-over-RTP.
    """
    import av
    import cv2
    import numpy as np

    container = av.open(rtsp_url)
    for frame in container.decode(video=0):
        img = frame.to_ndarray(format='bgr24')
        ret, jpg = cv2.imencode('.jpg', img)
        if not ret:
            continue
        mjpeg_frame = jpg.tobytes()
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n"
            b"Content-Length: " + str(len(mjpeg_frame)).encode() + b"\r\n\r\n" +
            mjpeg_frame + b"\r\n"
        )

# ============ API Endpoints ============

@app.post("/task")
async def manage_task(req: TaskRequest):
    # In a real implementation, interact with ROS API or system scripts here
    # Placeholder: Return JSON response
    return JSONResponse({"status": "success", "action": req.action, "script": req.script})

@app.get("/status")
async def get_status():
    data = await fetch_status()
    return JSONResponse(data)

@app.post("/move")
async def send_move(req: MoveRequest):
    # Prepare ROS message
    cmd = {
        "linear": {"x": req.linear_x, "y": req.linear_y, "z": req.linear_z},
        "angular": {"x": req.angular_x, "y": req.angular_y, "z": req.angular_z}
    }
    await ros_send_cmd_vel(cmd)
    return JSONResponse({"result": "command sent", "cmd": cmd})

@app.get("/video")
async def video_stream():
    async def streamer():
        async for chunk in rtsp_to_mjpeg_stream(RTSP_URL):
            yield chunk
    return StreamingResponse(streamer(), media_type="multipart/x-mixed-replace; boundary=frame")

# ============ Main Entrypoint ============
if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main:app", host=HTTP_SERVER_HOST, port=HTTP_SERVER_PORT, reload=False)