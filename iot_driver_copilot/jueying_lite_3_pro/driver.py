import os
import asyncio
import json
import base64
import cv2
import numpy as np
from typing import Dict, Any
from fastapi import FastAPI, Request, HTTPException
from fastapi.responses import StreamingResponse, JSONResponse
import uvicorn

# ======================= Environment Configuration =======================

DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
ROS_MASTER_URI = os.environ.get("ROS_MASTER_URI", f"http://{DEVICE_IP}:11311")
RTSP_URL = os.environ.get("RTSP_URL", f"rtsp://{DEVICE_IP}:8554/live")
HTTP_SERVER_HOST = os.environ.get("HTTP_SERVER_HOST", "0.0.0.0")
HTTP_SERVER_PORT = int(os.environ.get("HTTP_SERVER_PORT", 8080))
ROSBRIDGE_WS_URL = os.environ.get("ROSBRIDGE_WS_URL", f"ws://{DEVICE_IP}:9090")

# ======================= FastAPI App Setup =======================

app = FastAPI(title="Jueying Lite3 Pro Driver HTTP Server")

# ======================= RTSP to HTTP MJPEG Stream =======================

def mjpeg_stream(rtsp_url: str):
    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        raise HTTPException(status_code=503, detail="Unable to connect to RTSP stream")
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            _, jpeg = cv2.imencode('.jpg', frame)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
    finally:
        cap.release()

@app.get("/video", summary="HTTP-MJPEG proxy for RTSP video stream")
async def video_stream():
    return StreamingResponse(
        mjpeg_stream(RTSP_URL),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

# ======================= ROS/Status API =======================

# For demonstration: Simulate reading robot status data from ROS/UDP/JSON
async def fetch_status() -> Dict[str, Any]:
    # Here, you would use rospy or rosbridge websocket to get real data.
    # We'll mock this for now.
    return {
        "localization": {"x": 1.23, "y": 4.56, "theta": 0.78},
        "navigation": {"linear_vel": 0.1, "angular_vel": 0.01, "status": "idle"},
        "imu": {"x": 0.2, "y": -0.1, "z": 9.8},
        "joint_states": [{"name": "wheel_left", "pos": 0.5}, {"name": "wheel_right", "pos": 0.5}],
        "ultrasound_distance": [1.2, 1.5, 1.4],
        "handle_state": "neutral"
    }

@app.get("/status", summary="Fetch current status data including localization and navigation metrics")
async def status():
    data = await fetch_status()
    return JSONResponse(content=data)

# ======================= Move (cmd_vel) API =======================

async def send_move_command(vel_data: Dict[str, Any]):
    # Here you would publish to ROS (e.g., via rosbridge websocket or rospy)
    # We'll simulate success for now.
    return {"success": True, "sent_data": vel_data}

@app.post("/move", summary="Send movement commands to the robot")
async def move(request: Request):
    try:
        data = await request.json()
    except Exception:
        raise HTTPException(status_code=400, detail="Invalid JSON")
    result = await send_move_command(data)
    return JSONResponse(content=result)

# ======================= Task Management API =======================

async def manage_task(action: str, script_type: str):
    # Here you would start/stop scripts via ROS service/topic or SSH.
    # We'll simulate.
    if action not in ["start", "stop"]:
        return {"success": False, "error": "Invalid action"}
    return {"success": True, "action": action, "script_type": script_type}

@app.post("/task", summary="Manage operational scripts such as SLAM, LiDAR, or navigation")
async def task(request: Request):
    try:
        data = await request.json()
        action = data.get("action")
        script_type = data.get("script_type")
        if not (action and script_type):
            raise ValueError()
    except Exception:
        raise HTTPException(status_code=400, detail="Malformed JSON or missing required fields")
    result = await manage_task(action, script_type)
    return JSONResponse(content=result)

# ======================= Main Entrypoint =======================

if __name__ == "__main__":
    uvicorn.run("main:app", host=HTTP_SERVER_HOST, port=HTTP_SERVER_PORT, reload=False)