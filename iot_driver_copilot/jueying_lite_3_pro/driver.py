import os
import io
import json
import asyncio
import struct
from typing import Optional
from fastapi import FastAPI, Request, Response, BackgroundTasks, status
from fastapi.responses import StreamingResponse, JSONResponse, PlainTextResponse
from fastapi.middleware.cors import CORSMiddleware
import httpx
import uvicorn

# Environment Variables
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", "8080"))
ROS_API_PORT = int(os.environ.get("ROS_API_PORT", "9090"))  # For HTTP JSON bridge or similar if any
RTSP_PORT = int(os.environ.get("RTSP_PORT", "554"))
RTSP_PATH = os.environ.get("RTSP_PATH", "/stream1")
STATUS_UDP_PORT = int(os.environ.get("STATUS_UDP_PORT", "10001"))
COMMAND_UDP_PORT = int(os.environ.get("COMMAND_UDP_PORT", "10002"))

app = FastAPI(
    title="Jueying Lite3 Pro HTTP Driver",
    description="HTTP Server Driver for Jueying Lite3 Pro Mobile Robot Platform"
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --------- RTSP Proxy: RTSP -> HTTP MJPEG/Multipart Stream ---------
import av

async def rtsp_to_mjpeg_generator(rtsp_url):
    """Async generator converting RTSP H264 stream to multipart MJPEG for HTTP streaming."""
    try:
        container = av.open(rtsp_url, options={"rtsp_transport": "tcp"})
        video_stream = next(s for s in container.streams if s.type == 'video')
        for frame in container.decode(video=0):
            img = frame.to_image()
            buf = io.BytesIO()
            img.save(buf, format='JPEG')
            jpg_data = buf.getvalue()
            buf.close()
            boundary = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg_data + b"\r\n"
            yield boundary
    except Exception as e:
        yield b"--frame\r\nContent-Type: text/plain\r\n\r\nError: %s\r\n" % str(e).encode()

@app.get("/video")
async def video_stream():
    """
    HTTP MJPEG stream proxied from device's RTSP stream.
    """
    rtsp_url = f"rtsp://{DEVICE_IP}:{RTSP_PORT}{RTSP_PATH}"
    return StreamingResponse(
        rtsp_to_mjpeg_generator(rtsp_url),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

# --------- /move: UDP Command Forwarder ---------
import socket

def send_udp_cmd(data: bytes, port: int) -> bool:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.sendto(data, (DEVICE_IP, port))
        return True
    except Exception:
        return False

@app.post("/move")
async def move(request: Request):
    """
    Send movement commands to the robot via UDP.
    Accepts JSON payload, serializes, and sends to robot.
    """
    payload = await request.json()
    try:
        # Example: Expect {"linear": {"x": float, "y": float, "z": float}, "angular": {"x": float, "y": float, "z": float}}
        # Serialize to struct (example only, adjust to actual protocol)
        linear = payload.get("linear", {})
        angular = payload.get("angular", {})
        data = struct.pack(
            "ffffff",
            float(linear.get("x", 0)),
            float(linear.get("y", 0)),
            float(linear.get("z", 0)),
            float(angular.get("x", 0)),
            float(angular.get("y", 0)),
            float(angular.get("z", 0))
        )
        ok = send_udp_cmd(data, COMMAND_UDP_PORT)
        if not ok:
            return JSONResponse({"status": "error", "detail": "Failed to send UDP command"}, status_code=500)
        return {"status": "ok"}
    except Exception as e:
        return JSONResponse({"status": "error", "detail": str(e)}, status_code=400)

# --------- /task: Manage operational scripts (Start/Stop) ---------
# For demo, just UDP message with a JSON-encoded action+type
@app.post("/task")
async def task(request: Request):
    """
    Manage operational scripts such as SLAM, LiDAR, or navigation.
    JSON: {"action": "start"/"stop", "type": "slam"/"lidar"/"navigation"}
    """
    payload = await request.json()
    action = payload.get("action")
    type_ = payload.get("type")
    if action not in ("start", "stop") or not type_:
        return JSONResponse({"status": "error", "detail": "Invalid action or type"}, status_code=400)
    # Send UDP command as JSON
    msg = json.dumps({"action": action, "type": type_}).encode()
    ok = send_udp_cmd(msg, COMMAND_UDP_PORT)
    if not ok:
        return JSONResponse({"status": "error", "detail": "Failed to send UDP command"}, status_code=500)
    return {"status": "ok"}

# --------- /status: UDP Status Fetcher (One-shot) ---------
@app.get("/status")
async def status():
    """
    Fetch current status data including localization and navigation metrics via UDP (one-shot request-response).
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.settimeout(1.5)
            # Send a "status" request (protocol-specific, adjust as needed)
            s.sendto(b'status', (DEVICE_IP, STATUS_UDP_PORT))
            data, _ = s.recvfrom(8192)
            try:
                # Try to decode as JSON
                result = json.loads(data.decode())
                return JSONResponse(result)
            except Exception:
                # Return as plain text if not JSON
                return PlainTextResponse(data)
    except socket.timeout:
        return JSONResponse({"status": "error", "detail": "Timeout fetching status"}, status_code=504)
    except Exception as e:
        return JSONResponse({"status": "error", "detail": str(e)}, status_code=500)

if __name__ == "__main__":
    uvicorn.run(
        "main:app",
        host=SERVER_HOST,
        port=SERVER_PORT,
        reload=False
    )