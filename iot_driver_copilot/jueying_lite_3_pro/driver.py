import os
import io
import asyncio
import json
import base64
import socket
import struct
from typing import Optional
from fastapi import FastAPI, Request, Response, BackgroundTasks, HTTPException
from fastapi.responses import StreamingResponse, JSONResponse
from starlette.concurrency import run_in_threadpool
import cv2
import numpy as np

# Configuration from environment variables
ROBOT_IP = os.environ.get("DEVICE_IP", "192.168.1.100")
ROS_BRIDGE_PORT = int(os.environ.get("ROS_BRIDGE_PORT", "9090"))
UDP_PORT = int(os.environ.get("UDP_PORT", "5005"))
RTSP_URL = os.environ.get("RTSP_URL", f"rtsp://{ROBOT_IP}/live")  # device-specific default
HTTP_SERVER_HOST = os.environ.get("HTTP_SERVER_HOST", "0.0.0.0")
HTTP_SERVER_PORT = int(os.environ.get("HTTP_SERVER_PORT", "8080"))

app = FastAPI()

# ---- Video Stream Proxy (RTSP -> HTTP MJPEG) ----

def mjpeg_stream_from_rtsp(rtsp_url):
    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        raise HTTPException(status_code=502, detail="Could not connect to RTSP stream")
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            _, jpeg = cv2.imencode('.jpg', frame)
            jpg_bytes = jpeg.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpg_bytes + b'\r\n')
    finally:
        cap.release()

@app.get("/video")
async def video_feed():
    return StreamingResponse(mjpeg_stream_from_rtsp(RTSP_URL), media_type="multipart/x-mixed-replace; boundary=frame")

# ---- UDP Status Proxy (fetch latest sensor/status over UDP) ----

def fetch_udp_status():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0)
    try:
        # Protocol: Send a status request (customize as per device protocol)
        status_req = b'status'
        sock.sendto(status_req, (ROBOT_IP, UDP_PORT))
        data, _ = sock.recvfrom(65536)
        # Assume JSON encoded status over UDP
        try:
            status_json = json.loads(data.decode())
        except Exception:
            status_json = {"raw": base64.b64encode(data).decode()}
        return status_json
    except Exception as e:
        return {"error": str(e)}
    finally:
        sock.close()

@app.get("/status")
async def get_status():
    return JSONResponse(fetch_udp_status())

# ---- Movement Command Proxy (Send velocity command via UDP) ----

@app.post("/move")
async def move_robot(request: Request):
    payload = await request.json()
    # Assume payload includes fields for velocity (linear, angular, etc.)
    # The protocol below is illustrative; adapt to actual device command format
    cmd = json.dumps(payload).encode()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(cmd, (ROBOT_IP, UDP_PORT))
        return {"result": "command sent"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        sock.close()

# ---- Task Management Proxy (Start/Stop scripts via UDP) ----

@app.post("/task")
async def manage_task(request: Request):
    payload = await request.json()
    # Example: { "action": "start", "script": "slam" }
    cmd = json.dumps({"task": payload}).encode()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(cmd, (ROBOT_IP, UDP_PORT))
        return {"result": "task command sent"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        sock.close()

# ---- Main Entrypoint ----

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("driver:app", host=HTTP_SERVER_HOST, port=HTTP_SERVER_PORT, reload=False)