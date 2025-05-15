import os
import json
import asyncio
from aiohttp import web
import aiohttp
import cv2
import numpy as np

# Environment variables
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
RTSP_PORT = int(os.environ.get("RTSP_PORT", 554))
ROS_HOST = os.environ.get("ROS_HOST", "127.0.0.1")
ROS_PORT = int(os.environ.get("ROS_PORT", 9090))
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", 8080))
VIDEO_RTSP_PATH = os.environ.get("VIDEO_RTSP_PATH", "/stream1")
ROSBRIDGE_WS_URL = f"ws://{ROS_HOST}:{ROS_PORT}"

# --- HTTP Handler for /task ---
async def handle_task(request):
    data = await request.json()
    action = data.get("action")
    script_type = data.get("script_type")
    if not action or not script_type:
        return web.json_response({"error": "Missing 'action' or 'script_type'."}, status=400)

    # Publish to /script_control topic via ROSBridge WebSocket API
    payload = {
        "op": "publish",
        "topic": "/script_control",
        "msg": {
            "action": action,
            "script_type": script_type
        }
    }
    async with aiohttp.ClientSession() as session:
        async with session.ws_connect(ROSBRIDGE_WS_URL) as ws:
            await ws.send_json(payload)
            # Optionally wait for ack or just return success
            return web.json_response({"result": "requested"})

# --- HTTP Handler for /status ---
async def handle_status(request):
    # Subscribe and get one message from /robot_status topic via ROSBridge
    subscribe = {
        "op": "subscribe",
        "id": "1",
        "topic": "/robot_status"
    }
    unsubscribe = {
        "op": "unsubscribe",
        "id": "1",
        "topic": "/robot_status"
    }
    async with aiohttp.ClientSession() as session:
        async with session.ws_connect(ROSBRIDGE_WS_URL) as ws:
            await ws.send_json(subscribe)
            try:
                while True:
                    msg = await ws.receive(timeout=2)
                    if msg.type == aiohttp.WSMsgType.TEXT:
                        data = json.loads(msg.data)
                        if 'msg' in data:
                            await ws.send_json(unsubscribe)
                            return web.json_response(data['msg'])
            except asyncio.TimeoutError:
                await ws.send_json(unsubscribe)
                return web.json_response({"error": "No status data received."}, status=504)

# --- HTTP Handler for /move ---
async def handle_move(request):
    data = await request.json()
    cmd = data.get("cmd")
    if not cmd:
        return web.json_response({"error": "Missing 'cmd'."}, status=400)
    # Publish to /cmd_vel topic via ROSBridge
    payload = {
        "op": "publish",
        "topic": "/cmd_vel",
        "msg": cmd
    }
    async with aiohttp.ClientSession() as session:
        async with session.ws_connect(ROSBRIDGE_WS_URL) as ws:
            await ws.send_json(payload)
            return web.json_response({"result": "movement command sent"})

# --- MJPEG HTTP Video Proxy (/video) ---
async def mjpeg_video(request):
    boundary = "frame"
    resp = web.StreamResponse(
        status=200,
        reason='OK',
        headers={
            'Content-Type': f'multipart/x-mixed-replace; boundary=--{boundary}',
            'Cache-Control': 'no-cache',
            'Pragma': 'no-cache'
        }
    )
    await resp.prepare(request)

    rtsp_url = f"rtsp://{DEVICE_IP}:{RTSP_PORT}{VIDEO_RTSP_PATH}"
    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        await resp.write(b"--%s\r\nContent-Type: text/plain\r\n\r\nUnable to connect to RTSP stream.\r\n" % boundary.encode())
        await resp.write_eof()
        return resp

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            _, jpeg = cv2.imencode('.jpg', frame)
            img_bytes = jpeg.tobytes()
            await resp.write(b"--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % (boundary.encode(), len(img_bytes)))
            await resp.write(img_bytes)
            await resp.write(b"\r\n")
            await asyncio.sleep(0.04)  # ~25 FPS
    finally:
        cap.release()
        await resp.write_eof()
    return resp

# --- Application Setup ---
app = web.Application()
app.add_routes([
    web.post('/task', handle_task),
    web.get('/status', handle_status),
    web.post('/move', handle_move),
    web.get('/video', mjpeg_video)
])

if __name__ == '__main__':
    web.run_app(app, host=SERVER_HOST, port=SERVER_PORT)