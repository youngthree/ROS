import os
import asyncio
import json
from aiohttp import web, ClientSession
import aiohttp
import cv2
import numpy as np

# Environment variable configuration
DEVICE_IP = os.getenv('DEVICE_IP', '127.0.0.1')
ROS_API_URL = os.getenv('ROS_API_URL', f'http://{DEVICE_IP}:8080')
RTSP_URL = os.getenv('RTSP_URL', f'rtsp://{DEVICE_IP}/live')
SERVER_HOST = os.getenv('SERVER_HOST', '0.0.0.0')
SERVER_PORT = int(os.getenv('SERVER_PORT', '8000'))

# ---- Helper Functions ----

async def fetch_status():
    # Fetch localization & navigation status from ROS REST API or UDP endpoint (simulate as needed)
    url = f'{ROS_API_URL}/status'
    try:
        async with ClientSession() as session:
            async with session.get(url, timeout=3) as resp:
                if resp.status == 200:
                    return await resp.json()
                else:
                    return {"error": "Failed to fetch status", "code": resp.status}
    except Exception as e:
        return {"error": str(e)}

async def send_move_command(data):
    # Send velocity commands via ROS REST API (simulate as needed)
    url = f'{ROS_API_URL}/move'
    try:
        async with ClientSession() as session:
            async with session.post(url, json=data, timeout=3) as resp:
                if resp.status == 200:
                    return await resp.json()
                else:
                    return {"error": "Failed to send move command", "code": resp.status}
    except Exception as e:
        return {"error": str(e)}

async def manage_task(data):
    # Start/Stop SLAM/LIDAR/Navigation via ROS REST API (simulate as needed)
    url = f'{ROS_API_URL}/task'
    try:
        async with ClientSession() as session:
            async with session.post(url, json=data, timeout=3) as resp:
                if resp.status == 200:
                    return await resp.json()
                else:
                    return {"error": "Failed to manage task", "code": resp.status}
    except Exception as e:
        return {"error": str(e)}

# ---- HTTP Handlers ----

async def status_handler(request):
    result = await fetch_status()
    return web.json_response(result)

async def move_handler(request):
    try:
        data = await request.json()
    except Exception:
        return web.json_response({"error": "Invalid JSON"}, status=400)
    result = await send_move_command(data)
    return web.json_response(result)

async def task_handler(request):
    try:
        data = await request.json()
    except Exception:
        return web.json_response({"error": "Invalid JSON"}, status=400)
    result = await manage_task(data)
    return web.json_response(result)

async def mjpeg_stream_handler(request):
    # Proxy RTSP to HTTP MJPEG stream
    response = web.StreamResponse(
        status=200,
        reason='OK',
        headers={
            'Content-Type': 'multipart/x-mixed-replace; boundary=frame'
        }
    )
    await response.prepare(request)
    cap = cv2.VideoCapture(RTSP_URL)
    if not cap.isOpened():
        await response.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
        await response.write(b"\xff\xd8\xff\xe0" + b"NO VIDEO STREAM" + b"\xff\xd9")
        await response.write(b"\r\n")
        await response.write_eof()
        return response
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                await asyncio.sleep(0.05)
                continue
            ret, jpeg = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            img_bytes = jpeg.tobytes()
            await response.write(
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + img_bytes + b'\r\n'
            )
            await asyncio.sleep(0.04)  # ~25fps
    except asyncio.CancelledError:
        pass
    finally:
        cap.release()
        await response.write_eof()
    return response

# ---- Application ----

app = web.Application()
app.router.add_get('/status', status_handler)
app.router.add_post('/move', move_handler)
app.router.add_post('/task', task_handler)
app.router.add_get('/video', mjpeg_stream_handler)

if __name__ == "__main__":
    web.run_app(app, host=SERVER_HOST, port=SERVER_PORT)