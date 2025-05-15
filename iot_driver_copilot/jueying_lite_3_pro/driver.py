import os
import io
import json
import asyncio
from typing import Optional
from fastapi import FastAPI, Request, Response, BackgroundTasks, status
from fastapi.responses import StreamingResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
import aiohttp

# --- Environment Variables ---
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
DEVICE_ROSBRIDGE_PORT = int(os.environ.get("DEVICE_ROSBRIDGE_PORT", "9090"))  # For ROS API (websocket/json)
DEVICE_RTSP_PORT = int(os.environ.get("DEVICE_RTSP_PORT", "554"))  # For RTSP camera
DRIVER_HOST = os.environ.get("DRIVER_HOST", "0.0.0.0")
DRIVER_PORT = int(os.environ.get("DRIVER_PORT", "8080"))

# --- RTSP credentials (optional) ---
RTSP_USERNAME = os.environ.get("RTSP_USERNAME", "")
RTSP_PASSWORD = os.environ.get("RTSP_PASSWORD", "")
RTSP_PATH = os.environ.get("RTSP_PATH", "/stream")

# --- FastAPI setup ---
app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"], allow_credentials=True, allow_methods=["*"], allow_headers=["*"]
)

# --- Minimal ROS JSON API (rosbridge) client ---
import websockets  # stdlib asyncio websockets

async def ros_call(service: str, args: dict) -> dict:
    uri = f"ws://{DEVICE_IP}:{DEVICE_ROSBRIDGE_PORT}"
    async with websockets.connect(uri) as ws:
        req = {
            "op": "call_service",
            "service": service,
            "args": args,
            "id": "rosdriver"
        }
        await ws.send(json.dumps(req))
        async for msg in ws:
            data = json.loads(msg)
            if data.get("id") == "rosdriver" and data.get("op") == "service_response":
                return data.get("values", {})
    return {}

async def ros_publish(topic: str, msg_type: str, msg: dict):
    uri = f"ws://{DEVICE_IP}:{DEVICE_ROSBRIDGE_PORT}"
    async with websockets.connect(uri) as ws:
        req = {
            "op": "publish",
            "topic": topic,
            "msg": msg,
            "id": "rosdriver"
        }
        await ws.send(json.dumps(req))
        await asyncio.sleep(0.2)

async def ros_subscribe_once(topic: str):
    uri = f"ws://{DEVICE_IP}:{DEVICE_ROSBRIDGE_PORT}"
    async with websockets.connect(uri) as ws:
        subscribe_msg = {
            "op": "subscribe",
            "topic": topic,
            "id": "rosdriver"
        }
        await ws.send(json.dumps(subscribe_msg))
        async for msg in ws:
            data = json.loads(msg)
            if data.get("topic") == topic and data.get("op") == "publish":
                return data.get("msg", {})

# --- /task ---
@app.post("/task")
async def manage_task(request: Request):
    payload = await request.json()
    action = payload.get("action")
    script_type = payload.get("script_type")
    # In real device: map to ROS service or topic
    # Here: Just simulate as a ROS service call (you may need to adapt service name)
    service_name = f"/{script_type}_script_service"
    args = {"action": action}
    try:
        result = await ros_call(service_name, args)
        return JSONResponse({"result": result})
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=500)

# --- /status ---
@app.get("/status")
async def get_status():
    # Example: fetch from ROS topics
    status_data = {}
    try:
        status_data["localization"] = await ros_subscribe_once("/localization_status")
    except Exception:
        status_data["localization"] = None
    try:
        status_data["navigation"] = await ros_subscribe_once("/navigation_status")
    except Exception:
        status_data["navigation"] = None
    try:
        status_data["imu"] = await ros_subscribe_once("/imu/data")
    except Exception:
        status_data["imu"] = None
    try:
        status_data["joint_states"] = await ros_subscribe_once("/joint_states")
    except Exception:
        status_data["joint_states"] = None
    return status_data

# --- /move ---
@app.post("/move")
async def move(request: Request):
    payload = await request.json()
    # Assume payload is geometry_msgs/Twist
    try:
        await ros_publish("/cmd_vel", "geometry_msgs/Twist", payload)
        return {"success": True}
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=500)

# --- RTSP proxy to HTTP MJPEG ---
from PIL import Image
import cv2
import numpy as np

def build_rtsp_url():
    auth = ""
    if RTSP_USERNAME:
        auth = f"{RTSP_USERNAME}:{RTSP_PASSWORD}@" if RTSP_PASSWORD else f"{RTSP_USERNAME}@"
    return f"rtsp://{auth}{DEVICE_IP}:{DEVICE_RTSP_PORT}{RTSP_PATH}"

async def gen_mjpeg():
    rtsp_url = build_rtsp_url()
    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + b"\r\n"
        return
    while True:
        ret, frame = cap.read()
        if not ret:
            await asyncio.sleep(0.1)
            continue
        _, jpeg = cv2.imencode('.jpg', frame)
        frame_bytes = jpeg.tobytes()
        yield (
            b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n"
        )
        await asyncio.sleep(0.04)  # ~25 FPS

@app.get("/video")
async def video_stream():
    return StreamingResponse(gen_mjpeg(), media_type="multipart/x-mixed-replace; boundary=frame")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main:app", host=DRIVER_HOST, port=DRIVER_PORT, log_level="info")