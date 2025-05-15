import os
import json
import asyncio
import aiohttp
import aiohttp.web
import cv2
import numpy as np

# Environment variable configuration
ROBOT_IP = os.environ.get('ROBOT_IP', '127.0.0.1')
ROS_API_PORT = int(os.environ.get('ROS_API_PORT', '9090'))
RTSP_URL = os.environ.get('RTSP_URL', f'rtsp://{ROBOT_IP}:8554/live')  # Example RTSP
HTTP_SERVER_HOST = os.environ.get('HTTP_SERVER_HOST', '0.0.0.0')
HTTP_SERVER_PORT = int(os.environ.get('HTTP_SERVER_PORT', '8000'))

# Simulated ROS API endpoints (for /status and /move)
ROS_STATUS_URL = f'http://{ROBOT_IP}:{ROS_API_PORT}/status'
ROS_MOVE_URL = f'http://{ROBOT_IP}:{ROS_API_PORT}/move'
ROS_TASK_URL = f'http://{ROBOT_IP}:{ROS_API_PORT}/task'

routes = aiohttp.web.RouteTableDef()

@routes.post('/move')
async def move(request):
    # Forward the movement command to the robot's ROS API
    try:
        data = await request.json()
    except Exception:
        return aiohttp.web.json_response({'error': 'Invalid JSON'}, status=400)
    async with aiohttp.ClientSession() as session:
        try:
            async with session.post(ROS_MOVE_URL, json=data) as resp:
                result = await resp.json()
                return aiohttp.web.json_response(result, status=resp.status)
        except Exception as e:
            return aiohttp.web.json_response({'error': str(e)}, status=500)

@routes.get('/status')
async def status(request):
    # Fetch current status from the robot's ROS API
    async with aiohttp.ClientSession() as session:
        try:
            async with session.get(ROS_STATUS_URL) as resp:
                result = await resp.json()
                return aiohttp.web.json_response(result, status=resp.status)
        except Exception as e:
            return aiohttp.web.json_response({'error': str(e)}, status=500)

@routes.post('/task')
async def task(request):
    # Manage operational scripts (SLAM, LiDAR, navigation)
    try:
        data = await request.json()
    except Exception:
        return aiohttp.web.json_response({'error': 'Invalid JSON'}, status=400)
    async with aiohttp.ClientSession() as session:
        try:
            async with session.post(ROS_TASK_URL, json=data) as resp:
                result = await resp.json()
                return aiohttp.web.json_response(result, status=resp.status)
        except Exception as e:
            return aiohttp.web.json_response({'error': str(e)}, status=500)

@routes.get('/video')
async def video(request):
    # HTTP MJPEG stream from RTSP video
    response = aiohttp.web.StreamResponse(
        status=200,
        reason='OK',
        headers={
            'Content-Type': 'multipart/x-mixed-replace; boundary=frame'
        }
    )
    await response.prepare(request)
    # OpenCV VideoCapture for RTSP (no external commands)
    cap = cv2.VideoCapture(RTSP_URL)
    if not cap.isOpened():
        await response.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n')
        await response.write(b'')  # Empty JPEG
        await response.write(b'\r\n')
        await response.write_eof()
        return response
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            ret, jpeg = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            jpg_bytes = jpeg.tobytes()
            await response.write(
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + jpg_bytes + b'\r\n'
            )
            await asyncio.sleep(0.04)  # ~25fps
    finally:
        cap.release()
    await response.write_eof()
    return response

app = aiohttp.web.Application()
app.add_routes(routes)

if __name__ == '__main__':
    aiohttp.web.run_app(app, host=HTTP_SERVER_HOST, port=HTTP_SERVER_PORT)