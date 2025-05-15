import os
import io
import asyncio
import json
import cv2
import numpy as np
from fastapi import FastAPI, Request, Response, status
from fastapi.responses import StreamingResponse, JSONResponse, PlainTextResponse
from pydantic import BaseModel
import uvicorn
import aiohttp
from typing import Optional

# ======================== ENVIRONMENT VARIABLES ===========================
DEVICE_IP = os.getenv('DEVICE_IP')
DEVICE_ROS_API_PORT = int(os.getenv('DEVICE_ROS_API_PORT', '9090'))  # assumed rosbridge websocket port
DEVICE_RTSP_PORT = int(os.getenv('DEVICE_RTSP_PORT', '554'))
DEVICE_RTSP_PATH = os.getenv('DEVICE_RTSP_PATH', '/stream')  # example, update if needed
SERVER_HOST = os.getenv('SERVER_HOST', '0.0.0.0')
SERVER_PORT = int(os.getenv('SERVER_PORT', '8000'))

# ======================== FASTAPI APP =====================================
app = FastAPI(title="Jueying Lite3 Pro HTTP Driver")

# ======================== DATA MODELS =====================================
class TaskRequest(BaseModel):
    action: str  # "start" or "stop"
    script: str  # e.g., "SLAM", "LiDAR", "Navigation"

class MoveRequest(BaseModel):
    linear: dict
    angular: dict
    corrected: Optional[bool] = False

# =============== ROSBRIDGE CLIENT (websocket, JSON) =======================
# We'll use aiohttp for ROS bridge websocket communication
class ROSBridgeClient:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.ws = None
        self._id = 1

    async def connect(self):
        self.session = aiohttp.ClientSession()
        self.ws = await self.session.ws_connect(f'ws://{self.ip}:{self.port}')
    
    async def close(self):
        if self.ws:
            await self.ws.close()
        if self.session:
            await self.session.close()

    async def send(self, msg):
        if not self.ws:
            await self.connect()
        await self.ws.send_str(json.dumps(msg))
        resp = await self.ws.receive()
        if resp.type == aiohttp.WSMsgType.TEXT:
            return json.loads(resp.data)
        else:
            return None

    async def call_service(self, service, args=None):
        msg = {
            "op": "call_service",
            "service": service,
            "args": args or {},
            "id": str(self._id)
        }
        self._id += 1
        return await self.send(msg)

    async def publish(self, topic, msg_type, msg):
        msg = {
            "op": "publish",
            "topic": topic,
            "msg": msg,
            "type": msg_type,
            "id": str(self._id)
        }
        self._id += 1
        return await self.send(msg)

    async def subscribe_once(self, topic, msg_type):
        msg_id = str(self._id)
        subscribe_msg = {
            "op": "subscribe",
            "topic": topic,
            "type": msg_type,
            "id": msg_id,
            "throttle_rate": 0
        }
        await self.ws.send_str(json.dumps(subscribe_msg))
        while True:
            resp = await self.ws.receive()
            if resp.type == aiohttp.WSMsgType.TEXT:
                data = json.loads(resp.data)
                if data.get('topic') == topic:
                    unsub_msg = {
                        "op": "unsubscribe",
                        "topic": topic,
                        "id": msg_id
                    }
                    await self.ws.send_str(json.dumps(unsub_msg))
                    return data.get('msg', {})
            else:
                break
        return {}

# ======================== TASK MANAGEMENT =================================
@app.post("/task")
async def manage_task(req: TaskRequest):
    # Map script names to ROS service calls or topics
    script_map = {
        "SLAM": "/slam_control",
        "LiDAR": "/lidar_control",
        "Navigation": "/navigation_control"
    }
    script = req.script
    action = req.action
    if script not in script_map:
        return JSONResponse({"error": "Unknown script"}, status_code=400)
    ros = ROSBridgeClient(DEVICE_IP, DEVICE_ROS_API_PORT)
    await ros.connect()
    # We assume services follow the pattern: action: "start"/"stop"
    service = script_map[script]
    result = await ros.call_service(service, {"action": action})
    await ros.close()
    return JSONResponse(result)

# ======================== STATUS ENDPOINT =================================
@app.get("/status")
async def get_status():
    # Gather a set of status data from ROS topics
    ros = ROSBridgeClient(DEVICE_IP, DEVICE_ROS_API_PORT)
    await ros.connect()
    # Example topics to query (update as per actual robot topics)
    odom = await ros.subscribe_once("/leg_odom", "nav_msgs/Odometry")
    imu = await ros.subscribe_once("/imu_data", "sensor_msgs/Imu")
    handle = await ros.subscribe_once("/handle_state", "std_msgs/String")
    ultrasound = await ros.subscribe_once("/ultrasound_distance", "std_msgs/Float32MultiArray")
    navigation = await ros.subscribe_once("/navigation_status", "std_msgs/String")
    await ros.close()
    return JSONResponse({
        "leg_odom": odom,
        "imu": imu,
        "handle_state": handle,
        "ultrasound_distance": ultrasound,
        "navigation_status": navigation
    })

# ======================== MOVE ENDPOINT ===================================
@app.post("/move")
async def move_robot(req: MoveRequest):
    ros = ROSBridgeClient(DEVICE_IP, DEVICE_ROS_API_PORT)
    await ros.connect()
    topic = "/cmd_vel_corrected" if req.corrected else "/cmd_vel"
    msg = {
        "linear": req.linear,
        "angular": req.angular
    }
    result = await ros.publish(topic, "geometry_msgs/Twist", msg)
    await ros.close()
    return JSONResponse(result)

# ======================= MJPEG VIDEO PROXY ENDPOINT =======================
def bgr2jpeg(frame):
    ret, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    return jpeg.tobytes() if ret else None

async def rtsp_camera_stream():
    # OpenCV VideoCapture can read RTSP directly
    rtsp_url = f"rtsp://{DEVICE_IP}:{DEVICE_RTSP_PORT}{DEVICE_RTSP_PATH}"
    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        # Try to reconnect a few times
        for _ in range(5):
            await asyncio.sleep(1)
            cap.open(rtsp_url)
            if cap.isOpened():
                break
    if not cap.isOpened():
        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + bgr2jpeg(np.zeros((240,320,3),np.uint8)) + b"\r\n"
        return
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            jpeg = bgr2jpeg(frame)
            if not jpeg:
                continue
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n")
            await asyncio.sleep(0.04)  # ~25 fps
    finally:
        cap.release()

@app.get("/video")
async def video_feed():
    headers = {
        "Cache-Control": "no-cache",
        "Pragma": "no-cache"
    }
    return StreamingResponse(rtsp_camera_stream(), media_type="multipart/x-mixed-replace; boundary=frame", headers=headers)

# ======================== ROOT & HEALTHCHECK ==============================
@app.get("/")
async def root():
    return {"device": "Jueying Lite3 Pro", "status": "ok"}

@app.get("/health")
async def health():
    return PlainTextResponse("ok")

# ======================== MAIN ENTRY ======================================
if __name__ == "__main__":
    uvicorn.run("main:app", host=SERVER_HOST, port=SERVER_PORT, reload=False)