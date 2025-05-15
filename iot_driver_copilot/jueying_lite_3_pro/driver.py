import os
import json
import asyncio
import aiohttp
import aiohttp.web
import websockets
import cv2
import numpy as np

from aiohttp import web

# ================== Environment Variables ==================
DEVICE_IP = os.environ.get('DEVICE_IP', '127.0.0.1')
RTSP_PORT = int(os.environ.get('RTSP_PORT', '8554'))  # RTSP port of the camera/robot
RTSP_PATH = os.environ.get('RTSP_PATH', 'stream')     # RTSP stream path
HTTP_HOST = os.environ.get('HTTP_HOST', '0.0.0.0')
HTTP_PORT = int(os.environ.get('HTTP_PORT', '8080'))
ROS_STATUS_ENDPOINT = os.environ.get('ROS_STATUS_ENDPOINT', f'http://{DEVICE_IP}:5000/status')
ROS_MOVE_ENDPOINT = os.environ.get('ROS_MOVE_ENDPOINT', f'http://{DEVICE_IP}:5000/move')
ROS_TASK_ENDPOINT = os.environ.get('ROS_TASK_ENDPOINT', f'http://{DEVICE_IP}:5000/task')

# ================== RTSP to HTTP MJPEG Proxy ==================
class RTSPToHTTPMJPEG:
    def __init__(self, rtsp_url):
        self.rtsp_url = rtsp_url
        self.frame = None
        self.clients = set()
        self.running = False

    async def start(self):
        if not self.running:
            self.running = True
            loop = asyncio.get_event_loop()
            loop.create_task(self._capture_frames())

    async def _capture_frames(self):
        cap = cv2.VideoCapture(self.rtsp_url)
        if not cap.isOpened():
            print(f"Unable to open RTSP stream: {self.rtsp_url}")
            self.running = False
            return
        while self.running:
            ret, frame = cap.read()
            if not ret:
                await asyncio.sleep(0.1)
                continue
            _, jpeg = cv2.imencode('.jpg', frame)
            self.frame = jpeg.tobytes()
            await asyncio.sleep(0.033)  # ~30 FPS
        cap.release()

    async def mjpeg_handler(self, request):
        await self.start()
        response = web.StreamResponse(
            status=200,
            reason='OK',
            headers={
                'Content-Type': 'multipart/x-mixed-replace; boundary=frame'
            }
        )
        await response.prepare(request)
        try:
            while True:
                if self.frame is not None:
                    await response.write(b'--frame\r\n')
                    await response.write(b'Content-Type: image/jpeg\r\n\r\n')
                    await response.write(self.frame)
                    await response.write(b'\r\n')
                await asyncio.sleep(0.033)
        except asyncio.CancelledError:
            pass
        except Exception:
            pass
        return response

# ================== HTTP API Handlers ==================
async def handle_status(request):
    async with aiohttp.ClientSession() as session:
        async with session.get(ROS_STATUS_ENDPOINT) as resp:
            data = await resp.read()
            return web.Response(body=data, content_type='application/json')

async def handle_move(request):
    payload = await request.json()
    async with aiohttp.ClientSession() as session:
        async with session.post(ROS_MOVE_ENDPOINT, json=payload) as resp:
            data = await resp.read()
            return web.Response(body=data, content_type='application/json')

async def handle_task(request):
    payload = await request.json()
    async with aiohttp.ClientSession() as session:
        async with session.post(ROS_TASK_ENDPOINT, json=payload) as resp:
            data = await resp.read()
            return web.Response(body=data, content_type='application/json')

# ================== Main App Setup ==================
rtsp_url = f"rtsp://{DEVICE_IP}:{RTSP_PORT}/{RTSP_PATH}"
mjpeg_proxy = RTSPToHTTPMJPEG(rtsp_url)

app = web.Application()
app.router.add_get('/video', mjpeg_proxy.mjpeg_handler)
app.router.add_get('/status', handle_status)
app.router.add_post('/move', handle_move)
app.router.add_post('/task', handle_task)

if __name__ == '__main__':
    web.run_app(app, host=HTTP_HOST, port=HTTP_PORT)