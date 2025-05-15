import os
import io
import json
import threading
import time
from typing import Optional
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs

import socket
import struct

import cv2
import numpy as np

# -- Environment Variables --
ROBOT_IP = os.environ.get("ROBOT_IP", "127.0.0.1")
ROBOT_ROS_UDP_PORT = int(os.environ.get("ROBOT_ROS_UDP_PORT", "7070"))
ROBOT_RTSP_URL = os.environ.get("ROBOT_RTSP_URL", f"rtsp://{ROBOT_IP}/live")
HTTP_HOST = os.environ.get("HTTP_HOST", "0.0.0.0")
HTTP_PORT = int(os.environ.get("HTTP_PORT", "8080"))

# -- In-memory State --
STATUS_CACHE = {
    "localization": {},
    "navigation": {},
    "imu": {},
    "odometry": {},
    "ultrasound": {},
    "yolo": {},
    "last_update": 0
}
VELOCITY_QUEUE = []
SCRIPT_STATUS = {
    "SLAM": "stopped",
    "LiDAR": "stopped",
    "Navigation": "stopped"
}

# -- UDP Sensor Data Listener (Mock/Example for Status) --
def udp_sensor_listener():
    """
    Listen to UDP port for incoming sensor data and update STATUS_CACHE.
    This is a mockup; actual message decoding will depend on the robot's protocol.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ROBOT_IP, ROBOT_ROS_UDP_PORT))
    while True:
        try:
            data, _ = sock.recvfrom(65536)
            # Mock: suppose JSON messages
            try:
                msg = json.loads(data.decode("utf-8"))
                for k in msg:
                    STATUS_CACHE[k] = msg[k]
                STATUS_CACHE["last_update"] = time.time()
            except Exception:
                pass
        except Exception:
            continue

udp_thread = threading.Thread(target=udp_sensor_listener, daemon=True)
udp_thread.start()

# -- RTSP to HTTP JPEG Stream Proxy --
class RTSPProxy(threading.Thread):
    def __init__(self, rtsp_url):
        super().__init__(daemon=True)
        self.rtsp_url = rtsp_url
        self.last_frame = None
        self.running = threading.Event()
        self.running.set()
        self.capture = None

    def run(self):
        self.capture = cv2.VideoCapture(self.rtsp_url)
        while self.running.is_set():
            ret, frame = self.capture.read()
            if ret:
                ret2, jpeg = cv2.imencode('.jpg', frame)
                if ret2:
                    self.last_frame = jpeg.tobytes()
            else:
                # Try reconnecting if lost
                self.capture.release()
                time.sleep(1)
                self.capture = cv2.VideoCapture(self.rtsp_url)
            time.sleep(0.03)  # ~30fps

    def get_frame(self) -> Optional[bytes]:
        return self.last_frame

    def stop(self):
        self.running.clear()
        if self.capture:
            self.capture.release()

rtsp_proxy = RTSPProxy(ROBOT_RTSP_URL)
rtsp_proxy.start()

# -- HTTP Server --
class JueyingHTTPRequestHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    
    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/status":
            self.handle_status()
        elif parsed.path == "/video":
            self.handle_video_stream()
        else:
            self.send_response(404)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b'{"error": "Not found"}')

    def do_POST(self):
        parsed = urlparse(self.path)
        content_length = int(self.headers.get('Content-Length', 0))
        try:
            body = self.rfile.read(content_length)
            data = json.loads(body.decode('utf-8'))
        except Exception:
            data = {}
        if parsed.path == "/task":
            self.handle_task(data)
        elif parsed.path == "/move":
            self.handle_move(data)
        else:
            self.send_response(404)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b'{"error": "Not found"}')

    def handle_status(self):
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        data = {
            "localization": STATUS_CACHE.get("localization"),
            "navigation": STATUS_CACHE.get("navigation"),
            "imu": STATUS_CACHE.get("imu"),
            "odometry": STATUS_CACHE.get("odometry"),
            "ultrasound": STATUS_CACHE.get("ultrasound"),
            "yolo": STATUS_CACHE.get("yolo"),
            "script_status": SCRIPT_STATUS,
            "last_update": STATUS_CACHE.get("last_update")
        }
        self.wfile.write(json.dumps(data).encode("utf-8"))

    def handle_task(self, data):
        # data: {"action": "start"/"stop", "script_type": "SLAM"/"LiDAR"/"Navigation"}
        action = data.get("action")
        script_type = data.get("script_type")
        if action in ("start", "stop") and script_type in SCRIPT_STATUS:
            SCRIPT_STATUS[script_type] = "running" if action == "start" else "stopped"
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps({"result": "ok", "status": SCRIPT_STATUS}).encode("utf-8"))
        else:
            self.send_response(400)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b'{"error": "Invalid action or script_type"}')

    def handle_move(self, data):
        # data: {"linear": [x, y, z], "angular": [x, y, z]}
        VELOCITY_QUEUE.append((data, time.time()))
        # In a real driver, this would send to robot via UDP/ROS topic
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(json.dumps({"result": "ok"}).encode("utf-8"))

    def handle_video_stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        try:
            while True:
                frame = rtsp_proxy.get_frame()
                if not frame:
                    time.sleep(0.05)
                    continue
                self.wfile.write(b"--frame\r\n")
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(frame)))
                self.end_headers()
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                time.sleep(0.033)  # ~30fps
        except (BrokenPipeError, ConnectionResetError):
            pass

def run_server():
    server = HTTPServer((HTTP_HOST, HTTP_PORT), JueyingHTTPRequestHandler)
    print(f"HTTP server running at http://{HTTP_HOST}:{HTTP_PORT}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        rtsp_proxy.stop()

if __name__ == "__main__":
    run_server()