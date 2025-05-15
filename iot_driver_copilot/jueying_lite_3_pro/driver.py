import os
import io
import asyncio
import json
import cv2
import numpy as np
from aiohttp import web
from aiohttp import MultipartWriter

# -------------------- Environment Variables --------------------
DEVICE_IP = os.environ.get('DEVICE_IP', '127.0.0.1')
SERVER_HOST = os.environ.get('SERVER_HOST', '0.0.0.0')
SERVER_PORT = int(os.environ.get('SERVER_PORT', '8080'))
RTSP_PORT = int(os.environ.get('RTSP_PORT', '554'))
RTSP_PATH = os.environ.get('RTSP_PATH', '/stream1')
ROS_MASTER_URI = os.environ.get('ROS_MASTER_URI', f'http://{DEVICE_IP}:11311')
# (Other configs can be added as needed)

# -------------------- HTTP Server Handlers --------------------

routes = web.RouteTableDef()

# --------- 1. /task: Manage operational scripts (SLAM, LiDAR, Navigation) ---------
@routes.post('/task')
async def task_handler(request):
    try:
        payload = await request.json()
    except Exception:
        return web.json_response({'error': 'Invalid JSON'}, status=400)
    action = payload.get('action')
    script_type = payload.get('script_type')
    # Here should be integration with actual process management (e.g., ROS services/calls)
    # For now, simulate:
    result = {'status': 'ok', 'action': action, 'script_type': script_type}
    return web.json_response(result)

# --------- 2. /status: Fetch current status data (localization, navigation, sensors) ---------
@routes.get('/status')
async def status_handler(request):
    # Simulate status data (in real device, fetch from ROS, UDP, etc.)
    status_data = {
        'localization': {'x': 1.23, 'y': 3.21, 'theta': 0.123},
        'navigation_status': 'idle',
        'battery': 87.5,
        'imu': {'roll': 0.01, 'pitch': 0.02, 'yaw': 0.03},
        'ultrasound_distance': [0.5, 0.7, 1.2, 0.9],
        'yolov8_detection': [],
        'joint_states': {'joint1': 0.1, 'joint2': -0.2},
    }
    return web.json_response(status_data)

# --------- 3. /move: Send movement commands (velocity via JSON) ---------
@routes.post('/move')
async def move_handler(request):
    try:
        payload = await request.json()
    except Exception:
        return web.json_response({'error': 'Invalid JSON'}, status=400)
    # In real system, publish to ROS topic or send over UDP
    # Here, just echo back
    return web.json_response({'status': 'received', 'cmd': payload})

# --------- 4. /video: RTSP to HTTP MJPEG stream proxy ---------
@routes.get('/video')
async def video_handler(request):
    rtsp_url = f'rtsp://{DEVICE_IP}:{RTSP_PORT}{RTSP_PATH}'
    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        return web.Response(status=503, text='Unable to connect to RTSP stream')

    async def mjpeg_stream(writer):
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                ret, jpeg = cv2.imencode('.jpg', frame)
                if not ret:
                    continue
                img_bytes = jpeg.tobytes()
                await writer.write(
                    b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' +
                    img_bytes +
                    b'\r\n'
                )
                await asyncio.sleep(0.04)  # ~25 FPS
        finally:
            cap.release()

    resp = web.StreamResponse(
        status=200,
        reason='OK',
        headers={'Content-Type': 'multipart/x-mixed-replace; boundary=frame'}
    )
    await resp.prepare(request)
    await mjpeg_stream(resp)
    return resp

# -------------------- App Setup --------------------

app = web.Application()
app.add_routes(routes)

# -------------------- Main Entry --------------------

def main():
    web.run_app(app, host=SERVER_HOST, port=SERVER_PORT)

if __name__ == '__main__':
    main()