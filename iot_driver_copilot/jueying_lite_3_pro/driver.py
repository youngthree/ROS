import os
import asyncio
import json
from typing import Optional
from fastapi import FastAPI, HTTPException, Request, Response, status
from fastapi.responses import StreamingResponse, JSONResponse
import aiohttp
import cv2
import numpy as np

# Environment Variables for Configuration
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
ROS_API_URL = os.environ.get("ROS_API_URL", f"http://{DEVICE_IP}:5000")
RTSP_URL = os.environ.get("RTSP_URL", f"rtsp://{DEVICE_IP}/live")
HTTP_SERVER_HOST = os.environ.get("HTTP_SERVER_HOST", "0.0.0.0")
HTTP_SERVER_PORT = int(os.environ.get("HTTP_SERVER_PORT", "8080"))
RTSP_USERNAME = os.environ.get("RTSP_USERNAME", "")
RTSP_PASSWORD = os.environ.get("RTSP_PASSWORD", "")
# Only use RTSP and HTTP server ports because only those are needed for the driver.

app = FastAPI()

# Helper: MJPEG generator from RTSP stream using OpenCV
def mjpeg_stream(rtsp_url):
    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        cap.release()
        raise HTTPException(status_code=502, detail="Unable to open RTSP stream")
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            ret, jpeg = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
    finally:
        cap.release()

# API: Manage operational scripts (SLAM, LiDAR, navigation)
@app.post("/task")
async def manage_task(request: Request):
    try:
        data = await request.json()
        action = data.get('action')
        script_type = data.get('script_type')
        if not action or not script_type:
            raise HTTPException(status_code=400, detail="Missing 'action' or 'script_type'")
        # Example: POST to ROS API or internal script manager
        ros_url = f"{ROS_API_URL}/task"
        async with aiohttp.ClientSession() as session:
            async with session.post(ros_url, json={"action": action, "script_type": script_type}) as resp:
                resp_data = await resp.json()
                return JSONResponse(content=resp_data, status_code=resp.status)
    except Exception as ex:
        return JSONResponse(content={"error": str(ex)}, status_code=500)

# API: Fetch current status data (localization, navigation, sensor readings)
@app.get("/status")
async def get_status():
    try:
        ros_url = f"{ROS_API_URL}/status"
        async with aiohttp.ClientSession() as session:
            async with session.get(ros_url) as resp:
                if resp.status != 200:
                    raise HTTPException(status_code=resp.status, detail="Failed to fetch status")
                resp_data = await resp.json()
                return JSONResponse(content=resp_data)
    except Exception as ex:
        return JSONResponse(content={"error": str(ex)}, status_code=500)

# API: Send robot movement commands (cmd_vel, etc.)
@app.post("/move")
async def send_move(request: Request):
    try:
        data = await request.json()
        # Forward as-is to ROS API, adjust endpoint as needed for your ROS bridge.
        ros_url = f"{ROS_API_URL}/move"
        async with aiohttp.ClientSession() as session:
            async with session.post(ros_url, json=data) as resp:
                resp_data = await resp.json()
                return JSONResponse(content=resp_data, status_code=resp.status)
    except Exception as ex:
        return JSONResponse(content={"error": str(ex)}, status_code=500)

# API: Video streaming as MJPEG over HTTP (browser/CLI compatible)
@app.get("/video")
async def video_stream():
    # Build RTSP url with credentials if provided
    global RTSP_URL
    rtsp_url = RTSP_URL
    if RTSP_USERNAME and RTSP_PASSWORD and "@" not in RTSP_URL:
        proto, rest = RTSP_URL.split("://", 1)
        rtsp_url = f"{proto}://{RTSP_USERNAME}:{RTSP_PASSWORD}@{rest}"
    return StreamingResponse(mjpeg_stream(rtsp_url), media_type="multipart/x-mixed-replace; boundary=frame")

# Start server
if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main:app", host=HTTP_SERVER_HOST, port=HTTP_SERVER_PORT, reload=False)