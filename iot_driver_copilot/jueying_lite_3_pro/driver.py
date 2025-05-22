import os
import asyncio
import json
from fastapi import FastAPI, Request, Response, HTTPException
from fastapi.responses import StreamingResponse, JSONResponse
from pydantic import BaseModel
import socket
import aiohttp
import cv2
import numpy as np

# Environment variables for configuration
ROBOT_IP = os.environ.get("ROBOT_IP", "192.168.123.161")
ROBOT_UDP_PORT = int(os.environ.get("ROBOT_UDP_PORT", "10086"))
HTTP_SERVER_HOST = os.environ.get("HTTP_SERVER_HOST", "0.0.0.0")
HTTP_SERVER_PORT = int(os.environ.get("HTTP_SERVER_PORT", "8000"))
RTSP_URL = os.environ.get("ROBOT_RTSP_URL", f"rtsp://{ROBOT_IP}/live/stream")
RTSP_USERNAME = os.environ.get("ROBOT_RTSP_USERNAME", "")
RTSP_PASSWORD = os.environ.get("ROBOT_RTSP_PASSWORD", "")

# UDP command mapping
COMMANDS = {
    "/connect": b"connect",
    "/disconnet": b"disconnect",
    "/stand": b"stand",
    "/move_mode": b"move_mode",
    "/stationary_mode": b"stationary_mode",
    "/auto_mode": b"auto_mode",
    "/manual_mode": b"manual_mode",
    "/follow_mdoe": b"follow_mode",
    "/move_forward": b"move_forward",
    "/move_backward": b"move_backward",
    "/move_left": b"move_left",
    "/move_right": b"move_right",
    "/rotate_left": b"rotate_left",
    "/rotate_right": b"rotate_right",
    "/drive/stop": b"stop_movement",
    "/hello": b"greet",
    "/hop": b"jump",
    "/wiggle": b"twist_body",
    "/panic": b"emergency_stop",
}

app = FastAPI()

class Result(BaseModel):
    result: str

def send_udp_command(command: bytes) -> str:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2)
    try:
        sock.sendto(command, (ROBOT_IP, ROBOT_UDP_PORT))
        data, _ = sock.recvfrom(4096)
        return data.decode('utf-8', errors='ignore')
    except socket.timeout:
        return "timeout"
    except Exception as e:
        return f"error: {str(e)}"
    finally:
        sock.close()

async def async_send_udp_command(command: bytes) -> str:
    # To not block event loop, run in executor
    loop = asyncio.get_event_loop()
    return await loop.run_in_executor(None, send_udp_command, command)

@app.post("/connect")
async def connect():
    res = await async_send_udp_command(COMMANDS["/connect"])
    return JSONResponse(content={"result": res})

@app.post("/disconnet")
async def disconnect():
    res = await async_send_udp_command(COMMANDS["/disconnet"])
    return JSONResponse(content={"result": res})

@app.post("/stand")
async def stand():
    res = await async_send_udp_command(COMMANDS["/stand"])
    return JSONResponse(content={"result": res})

@app.post("/move_mode")
async def move_mode():
    res = await async_send_udp_command(COMMANDS["/move_mode"])
    return JSONResponse(content={"result": res})

@app.post("/stationary_mode")
async def stationary_mode():
    res = await async_send_udp_command(COMMANDS["/stationary_mode"])
    return JSONResponse(content={"result": res})

@app.post("/auto_mode")
async def auto_mode():
    res = await async_send_udp_command(COMMANDS["/auto_mode"])
    return JSONResponse(content={"result": res})

@app.post("/manual_mode")
async def manual_mode():
    res = await async_send_udp_command(COMMANDS["/manual_mode"])
    return JSONResponse(content={"result": res})

@app.post("/follow_mdoe")
async def follow_mode():
    res = await async_send_udp_command(COMMANDS["/follow_mdoe"])
    return JSONResponse(content={"result": res})

@app.post("/move_forward")
async def move_forward():
    res = await async_send_udp_command(COMMANDS["/move_forward"])
    return JSONResponse(content={"result": res})

@app.post("/move_backward")
async def move_backward():
    res = await async_send_udp_command(COMMANDS["/move_backward"])
    return JSONResponse(content={"result": res})

@app.post("/move_left")
async def move_left():
    res = await async_send_udp_command(COMMANDS["/move_left"])
    return JSONResponse(content={"result": res})

@app.post("/move_right")
async def move_right():
    res = await async_send_udp_command(COMMANDS["/move_right"])
    return JSONResponse(content={"result": res})

@app.post("/rotate_left")
async def rotate_left():
    res = await async_send_udp_command(COMMANDS["/rotate_left"])
    return JSONResponse(content={"result": res})

@app.post("/rotate_right")
async def rotate_right():
    res = await async_send_udp_command(COMMANDS["/rotate_right"])
    return JSONResponse(content={"result": res})

@app.post("/drive/stop")
async def stop_movement():
    res = await async_send_udp_command(COMMANDS["/drive/stop"])
    return JSONResponse(content={"result": res})

@app.post("/hello")
async def greet():
    res = await async_send_udp_command(COMMANDS["/hello"])
    return JSONResponse(content={"result": res})

@app.post("/hop")
async def jump():
    res = await async_send_udp_command(COMMANDS["/hop"])
    return JSONResponse(content={"result": res})

@app.post("/wiggle")
async def twist_body():
    res = await async_send_udp_command(COMMANDS["/wiggle"])
    return JSONResponse(content={"result": res})

@app.post("/panic")
async def emergency_stop():
    res = await async_send_udp_command(COMMANDS["/panic"])
    return JSONResponse(content={"result": res})

# --- RTSP to HTTP MJPEG Streaming ---

def get_rtsp_auth_url():
    if RTSP_USERNAME and RTSP_PASSWORD:
        # Insert username:password before host
        prefix = RTSP_URL.split("://")[0]
        rest = RTSP_URL.split("://")[1]
        return f"{prefix}://{RTSP_USERNAME}:{RTSP_PASSWORD}@{rest}"
    return RTSP_URL

def mjpeg_frame_generator():
    auth_url = get_rtsp_auth_url()
    cap = cv2.VideoCapture(auth_url)
    if not cap.isOpened():
        raise RuntimeError("Cannot open RTSP stream.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            ret, jpeg = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            jpg_bytes = jpeg.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpg_bytes + b'\r\n')
    finally:
        cap.release()

@app.get("/video")
def video_stream():
    return StreamingResponse(mjpeg_frame_generator(), media_type="multipart/x-mixed-replace; boundary=frame")

@app.get("/")
def root():
    return JSONResponse(content={
        "status": "ok",
        "robot_ip": ROBOT_IP,
        "udp_port": ROBOT_UDP_PORT,
        "commands": list(COMMANDS.keys()),
        "video_stream": "/video"
    })

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host=HTTP_SERVER_HOST,
        port=HTTP_SERVER_PORT,
        reload=False
    )