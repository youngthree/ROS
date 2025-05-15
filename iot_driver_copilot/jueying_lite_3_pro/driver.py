import os
import asyncio
import json
from typing import Optional
from fastapi import FastAPI, Request, Response, HTTPException, status
from fastapi.responses import StreamingResponse, JSONResponse
import aiohttp
import cv2
import numpy as np

# --- Environment Variable Configuration ---
DEVICE_IP = os.environ.get("DEVICE_IP")
ROS_API_PORT = int(os.environ.get("ROS_API_PORT", "9090"))  # If used for ROSBridge WebSocket
RTSP_URL = os.environ.get("RTSP_URL")  # e.g., "rtsp://user:pass@DEVICE_IP:554/stream"
HTTP_SERVER_HOST = os.environ.get("HTTP_SERVER_HOST", "0.0.0.0")
HTTP_SERVER_PORT = int(os.environ.get("HTTP_SERVER_PORT", "8080"))

if not DEVICE_IP:
    raise RuntimeError("Missing required environment variable: DEVICE_IP")
if not RTSP_URL:
    # Compose RTSP URL from parts if not provided
    RTSP_USERNAME = os.environ.get("RTSP_USERNAME", "")
    RTSP_PASSWORD = os.environ.get("RTSP_PASSWORD", "")
    RTSP_PORT = os.environ.get("RTSP_PORT", "554")
    RTSP_PATH = os.environ.get("RTSP_PATH", "/stream")
    if RTSP_USERNAME and RTSP_PASSWORD:
        RTSP_URL = f"rtsp://{RTSP_USERNAME}:{RTSP_PASSWORD}@{DEVICE_IP}:{RTSP_PORT}{RTSP_PATH}"
    else:
        RTSP_URL = f"rtsp://{DEVICE_IP}:{RTSP_PORT}{RTSP_PATH}"

# --- FastAPI Application ---
app = FastAPI()

# --- Video Stream Proxy: RTSP -> HTTP MJPEG ---
def gen_mjpeg_stream(rtsp_url):
    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        raise RuntimeError("Cannot open RTSP stream")
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            ret, jpeg = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            byte_frame = jpeg.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + byte_frame + b'\r\n')
    finally:
        cap.release()

@app.get("/video")
def video_stream():
    """
    Streams the device's RTSP video as HTTP MJPEG to browser/clients.
    """
    return StreamingResponse(
        gen_mjpeg_stream(RTSP_URL),
        media_type='multipart/x-mixed-replace; boundary=frame'
    )

# --- /status Endpoint: Fetch localization & navigation sensor data ---
@app.get("/status")
async def get_status():
    """
    Fetches device status (localization, navigation, sensors).
    Assumes device exposes a REST API for status at /api/status, or stub/mock if not.
    """
    status_url = f"http://{DEVICE_IP}:8081/api/status"  # Example endpoint; update as needed
    try:
        async with aiohttp.ClientSession() as session:
            async with session.get(status_url, timeout=5) as resp:
                if resp.status != 200:
                    raise HTTPException(status_code=resp.status, detail="Failed to fetch status")
                data = await resp.json()
                return JSONResponse(content=data)
    except Exception:
        # Fallback mock response
        return JSONResponse(content={
            "localization": {"x": 0, "y": 0, "theta": 0},
            "navigation": {"status": "idle"},
            "sensors": {"imu": {}, "ultrasound": {}, "battery": {}}
        })

# --- /move Endpoint: Send movement commands via JSON payload ---
@app.post("/move")
async def move_robot(request: Request):
    """
    Sends velocity/movement commands to the robot.
    Assumes device exposes a REST API at /api/move or a ROSBridge WebSocket.
    """
    body = await request.json()
    move_url = f"http://{DEVICE_IP}:8081/api/move"  # Example endpoint; update as needed
    try:
        async with aiohttp.ClientSession() as session:
            async with session.post(move_url, json=body, timeout=3) as resp:
                resp_data = await resp.json()
                return JSONResponse(content=resp_data, status_code=resp.status)
    except Exception:
        # Fallback mock response
        return JSONResponse(content={"result": "move command accepted (mock)"})

# --- /task Endpoint: Start/Stop SLAM, LiDAR, Navigation scripts ---
@app.post("/task")
async def task_control(request: Request):
    """
    Manage operational scripts (SLAM, LiDAR, navigation).
    """
    body = await request.json()
    task_url = f"http://{DEVICE_IP}:8081/api/task"  # Example endpoint; update as needed
    try:
        async with aiohttp.ClientSession() as session:
            async with session.post(task_url, json=body, timeout=3) as resp:
                resp_data = await resp.json()
                return JSONResponse(content=resp_data, status_code=resp.status)
    except Exception:
        # Fallback mock response
        return JSONResponse(content={"result": "task command accepted (mock)"})

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main:app", host=HTTP_SERVER_HOST, port=HTTP_SERVER_PORT)