import os
import io
import asyncio
import json
import base64
import cv2
import numpy as np
from typing import Optional
from fastapi import FastAPI, Request, Response, StreamingResponse, status
from fastapi.responses import JSONResponse
from pydantic import BaseModel
from starlette.concurrency import run_in_threadpool
import uvicorn

# ========== Environment Variables ==========
DEVICE_IP = os.getenv("DEVICE_IP", "127.0.0.1")
RTSP_URL = os.getenv("RTSP_URL", f"rtsp://{DEVICE_IP}/stream")
SERVER_HOST = os.getenv("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.getenv("SERVER_PORT", "8000"))
ROS_UDP_PORT = int(os.getenv("ROS_UDP_PORT", "15000"))
# Only use the above ports/protocols as required by the driver.

# ========== FastAPI App ==========
app = FastAPI()

# ========== Models ==========
class TaskRequest(BaseModel):
    action: str  # "start" or "stop"
    script_type: str  # "SLAM", "LiDAR", "navigation", etc.

class MoveRequest(BaseModel):
    linear_x: float
    linear_y: Optional[float] = 0.0
    linear_z: Optional[float] = 0.0
    angular_x: Optional[float] = 0.0
    angular_y: Optional[float] = 0.0
    angular_z: float

# ========== Internal Device State ==========
device_state = {
    "localization": {
        "x": 0.0,
        "y": 0.0,
        "theta": 0.0
    },
    "navigation": {
        "active": False,
        "target": None,
        "status": "idle"
    },
    "sensors": {
        "imu": {},
        "ultrasound_distance": [],
        "joint_states": {},
    },
    "operational_scripts": {
        "SLAM": False,
        "LiDAR": False,
        "navigation": False
    },
    "last_cmd_vel": {}
}

# ========== Dummy Simulated Device Interaction ==========

async def simulate_sensor_update():
    # In a real driver, this would subscribe to ROS topics or UDP streams.
    # Here, we simulate updates in the background.
    while True:
        device_state["localization"]["x"] += 0.01
        device_state["localization"]["y"] += 0.02
        device_state["localization"]["theta"] += 0.001
        device_state["navigation"]["status"] = "moving" if device_state["navigation"]["active"] else "idle"
        await asyncio.sleep(0.3)

@app.on_event("startup")
async def _startup():
    asyncio.create_task(simulate_sensor_update())

# ========== API Endpoints ==========

@app.post("/task")
async def manage_task(task: TaskRequest):
    t = task.script_type.upper()
    if t not in device_state["operational_scripts"]:
        return JSONResponse({"error": f"Unknown script type {t}"}, status_code=status.HTTP_400_BAD_REQUEST)
    if task.action.lower() == "start":
        device_state["operational_scripts"][t] = True
        return {"status": "started", "script_type": t}
    elif task.action.lower() == "stop":
        device_state["operational_scripts"][t] = False
        return {"status": "stopped", "script_type": t}
    else:
        return JSONResponse({"error": "Unknown action"}, status_code=status.HTTP_400_BAD_REQUEST)

@app.get("/status")
async def get_status():
    # Return localization, navigation, and sensor data
    return {
        "localization": device_state["localization"],
        "navigation": device_state["navigation"],
        "sensors": device_state["sensors"],
        "operational_scripts": device_state["operational_scripts"],
        "last_cmd_vel": device_state["last_cmd_vel"]
    }

@app.post("/move")
async def move_robot(move: MoveRequest):
    # Store command and simulate effect
    device_state["last_cmd_vel"] = move.dict()
    device_state["navigation"]["active"] = True
    return {"status": "moving", "received": move.dict()}

# ========== RTSP to HTTP MJPEG Streaming ==========

def mjpeg_generator(rtsp_url):
    # Open the RTSP stream using OpenCV
    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        raise RuntimeError("Cannot open RTSP stream.")
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            # Encode frame as JPEG in memory
            ret, jpeg = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            jpg_bytes = jpeg.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpg_bytes + b'\r\n')
    finally:
        cap.release()

@app.get("/video")
async def video_feed():
    # Stream as multipart/x-mixed-replace for browser MJPEG access
    return StreamingResponse(run_in_threadpool(mjpeg_generator, RTSP_URL),
                            media_type='multipart/x-mixed-replace; boundary=frame')

# ========== Main Entrypoint ==========
if __name__ == "__main__":
    uvicorn.run("main:app", host=SERVER_HOST, port=SERVER_PORT, reload=False)