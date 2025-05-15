import os
import io
import json
import asyncio
import threading
from fastapi import FastAPI, Request, Response, BackgroundTasks
from fastapi.responses import StreamingResponse, JSONResponse
from pydantic import BaseModel
from typing import Any, Dict, Optional
import uvicorn
import requests
import cv2
import numpy as np

# Environment Variables
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
RTSP_PORT = int(os.environ.get("RTSP_PORT", "554"))
RTSP_PATH = os.environ.get("RTSP_PATH", "live")
HTTP_SERVER_HOST = os.environ.get("HTTP_SERVER_HOST", "0.0.0.0")
HTTP_SERVER_PORT = int(os.environ.get("HTTP_SERVER_PORT", "8000"))
STATUS_API_PORT = int(os.environ.get("STATUS_API_PORT", "8080"))  # If REST API for status is on a different port

# RTSP URL construction (Jueying typically uses RTSP for video)
RTSP_URL = f"rtsp://{DEVICE_IP}:{RTSP_PORT}/{RTSP_PATH}"

app = FastAPI()

# ---- Models ----

class TaskCommand(BaseModel):
    action: str  # 'start' or 'stop'
    script_type: str  # 'SLAM', 'LiDAR', 'navigation', etc.

class MoveCommand(BaseModel):
    linear: Dict[str, float]  # e.g., {"x": 0.1, "y": 0.0, "z": 0.0}
    angular: Dict[str, float]  # e.g., {"x": 0.0, "y": 0.0, "z": 0.1}

# ---- Video Streaming ----

def mjpeg_generator(rtsp_url: str):
    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        _, jpeg = cv2.imencode('.jpg', frame)
        while True:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
    else:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            _, jpeg = cv2.imencode('.jpg', frame)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')

@app.get("/video")
def video_feed():
    return StreamingResponse(mjpeg_generator(RTSP_URL), media_type="multipart/x-mixed-replace; boundary=frame")

# ---- Task Control ----

@app.post("/task")
async def manage_task(cmd: TaskCommand):
    # Simulate or proxy to the device's API for script control
    # e.g., HTTP POST to the device's API endpoint, or ROS topic publish
    # Here we simulate a POST to a local REST API on the robot
    # You may replace this with actual device communication logic
    api_url = f"http://{DEVICE_IP}:{STATUS_API_PORT}/api/script"
    try:
        resp = requests.post(api_url, json={"action": cmd.action, "type": cmd.script_type}, timeout=3)
        return JSONResponse(content=resp.json(), status_code=resp.status_code)
    except Exception as e:
        return JSONResponse(content={"error": str(e)}, status_code=500)

# ---- Status Monitor ----

@app.get("/status")
async def status():
    # Simulate a GET to the device's status endpoint
    api_url = f"http://{DEVICE_IP}:{STATUS_API_PORT}/api/status"
    try:
        resp = requests.get(api_url, timeout=3)
        return JSONResponse(content=resp.json(), status_code=resp.status_code)
    except Exception as e:
        return JSONResponse(content={"error": str(e)}, status_code=500)

# ---- Movement Control ----

@app.post("/move")
async def move(cmd: MoveCommand):
    # Proxy movement command to the robot, e.g. via REST or ROS bridge
    api_url = f"http://{DEVICE_IP}:{STATUS_API_PORT}/api/move"
    payload = {
        "linear": cmd.linear,
        "angular": cmd.angular
    }
    try:
        resp = requests.post(api_url, json=payload, timeout=3)
        return JSONResponse(content=resp.json(), status_code=resp.status_code)
    except Exception as e:
        return JSONResponse(content={"error": str(e)}, status_code=500)

# ---- Application Entrypoint ----

if __name__ == "__main__":
    uvicorn.run(app, host=HTTP_SERVER_HOST, port=HTTP_SERVER_PORT)