import os
import io
import json
import asyncio
import aiohttp
import socket
import struct
import threading
from typing import Dict, Any
from aiohttp import web
import cv2
import numpy as np

# --- Environment Variables ---
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
ROS_UDP_PORT = int(os.environ.get("ROS_UDP_PORT", "9000"))
RTSP_URL = os.environ.get("RTSP_URL", f"rtsp://{DEVICE_IP}/video")
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", "8080"))

# --- ROS/UDP helpers ---
def parse_udp_packet(packet: bytes) -> Dict[str, Any]:
    # For demo: interpret as JSON. Adapt for real binary ROS/UDP protocols.
    try:
        return json.loads(packet.decode('utf-8'))
    except Exception:
        return {}

def udp_listener(status_cache):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((DEVICE_IP, ROS_UDP_PORT))
    while True:
        data, _ = sock.recvfrom(65535)
        parsed = parse_udp_packet(data)
        if parsed:
            status_cache.update(parsed)

status_cache = {}
udp_thread = threading.Thread(target=udp_listener, args=(status_cache,), daemon=True)
udp_thread.start()

# --- MJPEG HTTP stream from RTSP ---
def mjpeg_stream_generator(rtsp_url):
    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        yield (b'')
        return
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        _, jpeg = cv2.imencode('.jpg', frame)
        if not _:
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
    cap.release()

# --- HTTP Server ---
routes = web.RouteTableDef()

@routes.get('/status')
async def get_status(request):
    # Return a snapshot of the latest status_cache
    return web.json_response(status_cache)

@routes.post('/move')
async def post_move(request):
    # Expects JSON: {"linear": {"x": ..., "y": ..., ...}, "angular": {"z": ...}}
    data = await request.json()
    # Send velocity command over UDP in JSON for demo; adapt for ROS binary if needed.
    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd_json = json.dumps({
        "cmd_vel": data
    })
    cmd_sock.sendto(cmd_json.encode('utf-8'), (DEVICE_IP, ROS_UDP_PORT))
    cmd_sock.close()
    return web.json_response({"result": "sent", "data": data})

@routes.post('/task')
async def post_task(request):
    # Expects JSON: {"action": "start"/"stop", "script": "SLAM"/"LiDAR"/"Navigation"}
    data = await request.json()
    # For demo, just cache task state. In real device, trigger actual scripts.
    status_cache['tasks'] = status_cache.get('tasks', {})
    status_cache['tasks'][data.get("script")] = data.get("action")
    return web.json_response({"result": "ok", "updated": status_cache['tasks']})

@routes.get('/video')
async def video_stream(request):
    resp = web.StreamResponse(
        status=200,
        reason='OK',
        headers={
            'Content-Type': 'multipart/x-mixed-replace; boundary=frame'
        }
    )
    await resp.prepare(request)
    for frame in mjpeg_stream_generator(RTSP_URL):
        await resp.write(frame)
    return resp

# --- App Run ---
app = web.Application()
app.add_routes(routes)

if __name__ == '__main__':
    web.run_app(app, host=SERVER_HOST, port=SERVER_PORT)