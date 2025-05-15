import os
import asyncio
import json
from aiohttp import web, ClientSession
import aiohttp
import base64

# Environment Variables
DEVICE_IP = os.environ.get('DEVICE_IP', '127.0.0.1')
RTSP_URL = os.environ.get('RTSP_URL', f'rtsp://{DEVICE_IP}:8554/live')
ROS_API_URL = os.environ.get('ROS_API_URL', f'http://{DEVICE_IP}:8000')
HTTP_SERVER_HOST = os.environ.get('HTTP_SERVER_HOST', '0.0.0.0')
HTTP_SERVER_PORT = int(os.environ.get('HTTP_SERVER_PORT', '8080'))
RTSP_PROXY_PORT = int(os.environ.get('RTSP_PROXY_PORT', '8554'))

# MJPEG boundary
MJPEG_BOUNDARY = '--frame'

# Simple RTSP-to-MJPEG proxy (partial, naive implementation)
async def rtsp_to_mjpeg(rtsp_url):
    '''
    This generator yields JPEG frames extracted from an RTSP stream as multipart/x-mixed-replace.
    For simplicity, this stub expects the RTSP stream to provide JPEG frames via RTP,
    which is the case for some robots/cameras. For more complex codecs, a transcoder would be required.
    '''
    import cv2
    import numpy as np

    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        raise RuntimeError("Cannot open RTSP stream")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                await asyncio.sleep(0.1)
                continue
            ret, jpeg = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            jpg_bytes = jpeg.tobytes()
            # yield in multipart/x-mixed-replace format
            yield (
                f"{MJPEG_BOUNDARY}\r\n"
                "Content-Type: image/jpeg\r\n"
                f"Content-Length: {len(jpg_bytes)}\r\n\r\n"
            ).encode() + jpg_bytes + b"\r\n"
            await asyncio.sleep(0.04)  # ~25 FPS
    finally:
        cap.release()

async def mjpeg_stream(request):
    response = web.StreamResponse(
        status=200,
        reason='OK',
        headers={
            'Content-Type': f'multipart/x-mixed-replace; boundary={MJPEG_BOUNDARY[2:]}',
            'Cache-Control': 'no-cache, private',
            'Pragma': 'no-cache'
        }
    )
    await response.prepare(request)
    try:
        async for chunk in rtsp_to_mjpeg(RTSP_URL):
            await response.write(chunk)
    except Exception as e:
        pass
    finally:
        await response.write_eof()
    return response

async def get_status(request):
    async with ClientSession() as session:
        # Example: fetch status from ROS API or device HTTP API
        async with session.get(f"{ROS_API_URL}/status") as resp:
            if resp.status == 200:
                data = await resp.json()
                return web.json_response(data)
            else:
                return web.json_response({'error': 'Failed to fetch status'}, status=500)

async def post_task(request):
    payload = await request.json()
    action = payload.get('action')  # 'start' or 'stop'
    script_type = payload.get('script_type')  # e.g., 'slam', 'lidar', 'navigation'
    cmd = {
        'action': action,
        'script_type': script_type
    }
    async with ClientSession() as session:
        async with session.post(f"{ROS_API_URL}/task", json=cmd) as resp:
            if resp.status == 200:
                data = await resp.json()
                return web.json_response(data)
            else:
                return web.json_response({'error': 'Failed to manage task'}, status=500)

async def post_move(request):
    payload = await request.json()
    # Forward to ROS API or relevant robot endpoint
    async with ClientSession() as session:
        async with session.post(f"{ROS_API_URL}/move", json=payload) as resp:
            if resp.status == 200:
                data = await resp.json()
                return web.json_response(data)
            else:
                return web.json_response({'error': 'Failed to move'}, status=500)

routes = [
    web.get('/video.mjpg', mjpeg_stream),
    web.get('/status', get_status),
    web.post('/task', post_task),
    web.post('/move', post_move),
]

app = web.Application()
app.add_routes(routes)

if __name__ == '__main__':
    web.run_app(app, host=HTTP_SERVER_HOST, port=HTTP_SERVER_PORT)