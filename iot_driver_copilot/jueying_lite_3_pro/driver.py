import os
import asyncio
import json
import aiohttp
from aiohttp import web
import websockets
import base64

# Environment variables/config
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
DEVICE_RTSP_PORT = int(os.environ.get("DEVICE_RTSP_PORT", "554"))
DEVICE_ROSBRIDGE_PORT = int(os.environ.get("DEVICE_ROSBRIDGE_PORT", "9090"))
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", "8080"))
RTSP_STREAM_PATH = os.environ.get("RTSP_STREAM_PATH", "/video")  # e.g., /live/stream
RTSP_USERNAME = os.environ.get("RTSP_USERNAME", "")
RTSP_PASSWORD = os.environ.get("RTSP_PASSWORD", "")

# ROSBridge websocket URL
ROSBRIDGE_URI = f"ws://{DEVICE_IP}:{DEVICE_ROSBRIDGE_PORT}"

routes = web.RouteTableDef()

async def rosbridge_call(service, args=None):
    async with websockets.connect(ROSBRIDGE_URI) as ws:
        call_id = "call_" + str(os.urandom(8).hex())
        request = {
            "op": "call_service",
            "service": service,
            "args": args or {},
            "id": call_id,
        }
        await ws.send(json.dumps(request))
        async for msg in ws:
            resp = json.loads(msg)
            if resp.get("id") == call_id and resp.get("op") == "service_response":
                return resp.get("values", {})
    return {}

async def rosbridge_publish(topic, msg_type, msg):
    async with websockets.connect(ROSBRIDGE_URI) as ws:
        request = {
            "op": "publish",
            "topic": topic,
            "msg": msg,
        }
        await ws.send(json.dumps(request))
        # Don't wait for response, fire and forget

async def rosbridge_subscribe(topic, msg_type, count=1):
    async with websockets.connect(ROSBRIDGE_URI) as ws:
        sub_id = "sub_" + str(os.urandom(8).hex())
        request = {
            "op": "subscribe",
            "topic": topic,
            "type": msg_type,
            "id": sub_id,
        }
        await ws.send(json.dumps(request))
        results = []
        for _ in range(count):
            msg = await ws.recv()
            resp = json.loads(msg)
            if resp.get("topic") == topic:
                results.append(resp.get("msg"))
        unsub_req = {"op": "unsubscribe", "id": sub_id, "topic": topic}
        await ws.send(json.dumps(unsub_req))
        return results

@routes.post('/task')
async def manage_task(request):
    data = await request.json()
    action = data.get('action')
    script_type = data.get('script_type')
    # Example: start/stop SLAM, LiDAR, Navigation
    # Let's assume custom ROS services: /start_script, /stop_script
    if action not in ['start', 'stop'] or not script_type:
        return web.json_response({'error': 'Invalid payload'}, status=400)
    service = f"/{action}_script"
    args = {"script_type": script_type}
    try:
        resp = await rosbridge_call(service, args)
        return web.json_response({"result": "ok", "response": resp})
    except Exception as e:
        return web.json_response({'error': str(e)}, status=500)

@routes.get('/status')
async def get_status(request):
    # Example: subscribe once to sensor topics
    out = {}
    try:
        # Localization
        loc = await rosbridge_subscribe('/localization', 'geometry_msgs/PoseWithCovarianceStamped', 1)
        # Navigation status
        nav = await rosbridge_subscribe('/navigation_status', 'std_msgs/String', 1)
        # IMU
        imu = await rosbridge_subscribe('/imu', 'sensor_msgs/Imu', 1)
        # Odom
        odom = await rosbridge_subscribe('/odom', 'nav_msgs/Odometry', 1)
        out = {
            "localization": loc[0] if loc else {},
            "navigation_status": nav[0] if nav else {},
            "imu": imu[0] if imu else {},
            "odom": odom[0] if odom else {},
        }
    except Exception as e:
        return web.json_response({'error': str(e)}, status=500)
    return web.json_response(out)

@routes.post('/move')
async def move(request):
    data = await request.json()
    # Move command e.g. { "linear": { "x": 0.5, "y": 0, "z": 0 }, "angular": { "z": 0.2 } }
    try:
        await rosbridge_publish('/cmd_vel', 'geometry_msgs/Twist', data)
        return web.json_response({"result": "ok"})
    except Exception as e:
        return web.json_response({'error': str(e)}, status=500)

@routes.get('/video')
async def video_proxy(request):
    """
    HTTP MJPEG proxy for RTSP stream, accessible from browser.
    """
    import cv2

    # Build RTSP URI
    if RTSP_USERNAME and RTSP_PASSWORD:
        auth = f"{RTSP_USERNAME}:{RTSP_PASSWORD}@"
    else:
        auth = ""
    rtsp_url = f"rtsp://{auth}{DEVICE_IP}:{DEVICE_RTSP_PORT}{RTSP_STREAM_PATH}"

    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        return web.Response(status=503, text="Could not open RTSP stream")
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
            ret, frame = cap.read()
            if not ret:
                break
            ret, jpg = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            await response.write(b"--frame\r\n")
            await response.write(b"Content-Type: image/jpeg\r\n\r\n")
            await response.write(jpg.tobytes())
            await response.write(b"\r\n")
            await asyncio.sleep(0.04)  # ~25fps
    except asyncio.CancelledError:
        pass
    finally:
        cap.release()
    return response

app = web.Application()
app.add_routes(routes)

if __name__ == "__main__":
    web.run_app(app, host=SERVER_HOST, port=SERVER_PORT)