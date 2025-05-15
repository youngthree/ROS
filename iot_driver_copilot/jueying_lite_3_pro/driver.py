import os
import asyncio
import json
import uvicorn
from fastapi import FastAPI, Request, Response, HTTPException
from fastapi.responses import StreamingResponse, JSONResponse
from typing import Any, Dict
import aiohttp

# Environment variables for config
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
DEVICE_ROS_PORT = int(os.environ.get("DEVICE_ROS_PORT", "9090"))
DEVICE_RTSP_PORT = int(os.environ.get("DEVICE_RTSP_PORT", "554"))
HTTP_SERVER_HOST = os.environ.get("HTTP_SERVER_HOST", "0.0.0.0")
HTTP_SERVER_PORT = int(os.environ.get("HTTP_SERVER_PORT", "8000"))
RTSP_PATH = os.environ.get("RTSP_PATH", "/stream")  # Path to camera stream

# ROS/UDP/mock config (should be replaced with real robot interface)
# For demonstration, this will simulate robot state & actions.

app = FastAPI()

robot_state = {
    "localization": {"x": 1.1, "y": 2.2, "theta": 0.3},
    "navigation": {"active": False, "goal": None},
    "imu": {"roll": 0.01, "pitch": 0.02, "yaw": 0.03},
    "velocity": {"x": 0.0, "y": 0.0, "z": 0.0}
}
task_status = {"SLAM": "stopped", "LiDAR": "stopped", "Navigation": "stopped"}

@app.post("/task")
async def manage_task(request: Request):
    """
    Manage operational scripts such as SLAM, LiDAR, or navigation.
    Body: {"action": "start"/"stop", "script": "SLAM"/"LiDAR"/"Navigation"}
    """
    data = await request.json()
    action = data.get("action")
    script = data.get("script")
    if script not in task_status or action not in ("start", "stop"):
        raise HTTPException(status_code=400, detail="Invalid script or action")
    task_status[script] = "running" if action == "start" else "stopped"
    return JSONResponse({"status": "ok", "script": script, "state": task_status[script]})

@app.get("/status")
async def get_status():
    """
    Fetch current status data including localization and navigation metrics.
    """
    # Replace with actual ROS/UDP fetch in real driver
    return JSONResponse({
        "localization": robot_state["localization"],
        "navigation": robot_state["navigation"],
        "imu": robot_state["imu"],
        "velocity": robot_state["velocity"],
        "tasks": task_status
    })

@app.post("/move")
async def move_robot(request: Request):
    """
    Send movement commands to the robot.
    Body: {"cmd_vel": {"x": float, "y": float, "z": float}}
    """
    data = await request.json()
    cmd_vel = data.get("cmd_vel")
    if not isinstance(cmd_vel, dict):
        raise HTTPException(status_code=400, detail="Missing cmd_vel")
    robot_state["velocity"] = cmd_vel
    # Send to ROS topic or UDP here in real driver
    return JSONResponse({"status": "ok", "cmd_vel": cmd_vel})

# RTSP-to-HTTP proxy endpoint
@app.get("/video")
async def video():
    """
    Proxy the robot's RTSP video stream as HTTP MJPEG for browser/command-line access.
    """
    rtsp_url = f"rtsp://{DEVICE_IP}:{DEVICE_RTSP_PORT}{RTSP_PATH}"
    return StreamingResponse(
        rtsp_to_mjpeg(rtsp_url),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

async def rtsp_to_mjpeg(rtsp_url):
    """
    Connects to RTSP, decodes frames, encodes as JPEG, yields multipart MJPEG.
    This implementation uses aiortc for RTSP and OpenCV for decoding/encoding.
    """
    import cv2
    import numpy as np

    # aiortc is not used here due to third-party command execution restriction.
    # Instead, using OpenCV VideoCapture which links directly to the device.
    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
        await asyncio.sleep(0.1)
        return

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                await asyncio.sleep(0.1)
                continue
            _, jpeg = cv2.imencode('.jpg', frame)
            frame_bytes = jpeg.tobytes()
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n"
                + frame_bytes +
                b"\r\n"
            )
            await asyncio.sleep(0.04)  # ~25fps
    finally:
        cap.release()

if __name__ == "__main__":
    uvicorn.run(
        "main:app",
        host=HTTP_SERVER_HOST,
        port=HTTP_SERVER_PORT,
        reload=False,
        access_log=True
    )