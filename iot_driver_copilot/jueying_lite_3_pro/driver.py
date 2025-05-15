import os
import asyncio
import json
from fastapi import FastAPI, Request, Response, status
from fastapi.responses import StreamingResponse, JSONResponse
import uvicorn
import aiohttp
import cv2
import numpy as np

# Configuration from environment variables
DEVICE_IP = os.environ.get('DEVICE_IP', '127.0.0.1')
SERVER_HOST = os.environ.get('SERVER_HOST', '0.0.0.0')
SERVER_PORT = int(os.environ.get('SERVER_PORT', 8080))
RTSP_PORT = int(os.environ.get('RTSP_PORT', 554))
RTSP_PATH = os.environ.get('RTSP_PATH', '/live')  # e.g., /live or /stream
RTSP_USER = os.environ.get('RTSP_USER', '')
RTSP_PASS = os.environ.get('RTSP_PASS', '')

# ROS/Status/Command endpoints (should be replaced with the actual robot's API endpoints)
ROBOT_API_BASE = os.environ.get('ROBOT_API_BASE', f'http://{DEVICE_IP}:8808')

app = FastAPI()


@app.post("/task")
async def manage_task(request: Request):
    try:
        payload = await request.json()
    except Exception:
        return JSONResponse({"error": "Invalid JSON"}, status_code=status.HTTP_400_BAD_REQUEST)
    # Proxy the request to the robot's REST API (or mock if unavailable)
    async with aiohttp.ClientSession() as session:
        try:
            async with session.post(f"{ROBOT_API_BASE}/task", json=payload) as resp:
                data = await resp.text()
                return Response(content=data, status_code=resp.status, media_type=resp.headers.get("content-type", "application/json"))
        except Exception as e:
            return JSONResponse({"error": str(e)}, status_code=500)


@app.get("/status")
async def get_status():
    # Proxy the status request
    async with aiohttp.ClientSession() as session:
        try:
            async with session.get(f"{ROBOT_API_BASE}/status") as resp:
                data = await resp.text()
                return Response(content=data, status_code=resp.status, media_type=resp.headers.get("content-type", "application/json"))
        except Exception as e:
            return JSONResponse({"error": str(e)}, status_code=500)


@app.post("/move")
async def move_robot(request: Request):
    try:
        payload = await request.json()
    except Exception:
        return JSONResponse({"error": "Invalid JSON"}, status_code=status.HTTP_400_BAD_REQUEST)
    # Proxy the movement command
    async with aiohttp.ClientSession() as session:
        try:
            async with session.post(f"{ROBOT_API_BASE}/move", json=payload) as resp:
                data = await resp.text()
                return Response(content=data, status_code=resp.status, media_type=resp.headers.get("content-type", "application/json"))
        except Exception as e:
            return JSONResponse({"error": str(e)}, status_code=500)


def get_rtsp_url():
    if RTSP_USER and RTSP_PASS:
        return f"rtsp://{RTSP_USER}:{RTSP_PASS}@{DEVICE_IP}:{RTSP_PORT}{RTSP_PATH}"
    return f"rtsp://{DEVICE_IP}:{RTSP_PORT}{RTSP_PATH}"


async def mjpeg_stream():
    loop = asyncio.get_event_loop()
    rtsp_url = get_rtsp_url()
    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        raise RuntimeError("Unable to open RTSP stream")
    try:
        while True:
            ret, frame = await loop.run_in_executor(None, cap.read)
            if not ret:
                await asyncio.sleep(0.1)
                continue
            _, jpeg = cv2.imencode('.jpg', frame)
            byte_frame = jpeg.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + byte_frame + b'\r\n')
            await asyncio.sleep(0.04)  # ~25fps
    finally:
        cap.release()


@app.get("/video")
async def video_feed():
    return StreamingResponse(mjpeg_stream(), media_type='multipart/x-mixed-replace; boundary=frame')


if __name__ == "__main__":
    uvicorn.run(app, host=SERVER_HOST, port=SERVER_PORT)