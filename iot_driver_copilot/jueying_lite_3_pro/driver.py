import os
import io
import sys
import cv2
import json
import time
import queue
import base64
import socket
import struct
import asyncio
import threading
from typing import Optional, Dict, Any
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs

# Environment variables (all must be set)
DEVICE_IP = os.environ.get('DEVICE_IP', '127.0.0.1')
RTSP_PORT = int(os.environ.get('RTSP_PORT', '554'))
ROS_UDP_PORT = int(os.environ.get('ROS_UDP_PORT', '15000'))
SERVER_HOST = os.environ.get('SERVER_HOST', '0.0.0.0')
SERVER_PORT = int(os.environ.get('SERVER_PORT', '8080'))
RTSP_PATH = os.environ.get('RTSP_PATH', '/stream')
RTSP_USER = os.environ.get('RTSP_USER', '')
RTSP_PASS = os.environ.get('RTSP_PASS', '')

# Global queues for UDP and status data
udp_data_queue = queue.Queue()
status_data = {
    "localization": {},
    "navigation": {},
    "imu": {},
    "joint_states": {},
    "handle_state": {},
    "ultrasound_distance": {},
    "yolov8": {},
    "video_stream": {},
    "operation_scripts": {}
}


def get_rtsp_url():
    userinfo = f"{RTSP_USER}:{RTSP_PASS}@" if RTSP_USER and RTSP_PASS else ""
    return f"rtsp://{userinfo}{DEVICE_IP}:{RTSP_PORT}{RTSP_PATH}"


def udp_receiver_thread(port: int, data_queue: queue.Queue):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((DEVICE_IP, port))
    while True:
        data, addr = sock.recvfrom(65535)
        try:
            message = data.decode('utf-8')
            parsed = json.loads(message)
            data_queue.put(parsed)
        except Exception:
            continue


def update_status():
    # Pop from the queue and update status_data (simulate real update)
    while True:
        try:
            data = udp_data_queue.get(timeout=1)
            for key in status_data:
                if key in data:
                    status_data[key] = data[key]
        except queue.Empty:
            pass
        time.sleep(0.1)


def ros_publish(topic: str, message: dict):
    # For demonstration, this just records the command in status_data
    # In a real implementation, this would interface with ROS topics.
    status_data["last_command"] = {"topic": topic, "message": message, "time": time.time()}


def handle_operation_script(action: str, script_type: str):
    # Simulate starting/stopping scripts
    if action == "start":
        status_data["operation_scripts"][script_type] = "running"
    elif action == "stop":
        status_data["operation_scripts"][script_type] = "stopped"
    else:
        status_data["operation_scripts"][script_type] = "unknown"


class StreamingOutput:
    def __init__(self):
        self.frame = None
        self.buffer = io.BytesIO()
        self.condition = threading.Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()

    def get_frame(self):
        with self.condition:
            self.condition.wait()
            return self.frame


class RobotHTTPRequestHandler(BaseHTTPRequestHandler):
    def _set_headers(self, status=HTTPStatus.OK, content_type="application/json"):
        self.send_response(status)
        self.send_header('Content-type', content_type)
        self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/status":
            self._set_headers()
            self.wfile.write(json.dumps(status_data).encode('utf-8'))
        elif parsed.path == "/video":
            # HTTP MJPEG video streaming from RTSP
            self.send_response(200)
            self.send_header('Age', '0')
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                for jpeg in mjpeg_stream_generator():
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', str(len(jpeg)))
                    self.end_headers()
                    self.wfile.write(jpeg)
                    self.wfile.write(b'\r\n')
            except Exception:
                pass
        else:
            self._set_headers(HTTPStatus.NOT_FOUND)
            self.wfile.write(json.dumps({"error": "Not found"}).encode('utf-8'))

    def do_POST(self):
        parsed = urlparse(self.path)
        content_length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(content_length)
        try:
            data = json.loads(body.decode('utf-8'))
        except Exception:
            self._set_headers(HTTPStatus.BAD_REQUEST)
            self.wfile.write(json.dumps({"error": "Invalid JSON"}).encode('utf-8'))
            return

        if parsed.path == "/move":
            # Accepts {"linear": {"x": ..., "y": ..., "z": ...}, "angular": {"x": ..., "y": ..., "z": ...}}
            ros_publish('cmd_vel', data)
            self._set_headers()
            self.wfile.write(json.dumps({"status": "ok"}).encode('utf-8'))
        elif parsed.path == "/task":
            # Accepts {"action": "start"/"stop", "script_type": "SLAM"/"LiDAR"/"navigation"}
            action = data.get('action')
            script_type = data.get('script_type')
            if action and script_type:
                handle_operation_script(action, script_type)
                self._set_headers()
                self.wfile.write(json.dumps({"status": "ok"}).encode('utf-8'))
            else:
                self._set_headers(HTTPStatus.BAD_REQUEST)
                self.wfile.write(json.dumps({"error": "Missing action or script_type"}).encode('utf-8'))
        else:
            self._set_headers(HTTPStatus.NOT_FOUND)
            self.wfile.write(json.dumps({"error": "Not found"}).encode('utf-8'))


def mjpeg_stream_generator():
    rtsp_url = get_rtsp_url()
    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        # Yield a placeholder image if stream is not available
        img = 255 * np.ones((480, 640, 3), np.uint8)
        ret, jpeg = cv2.imencode('.jpg', img)
        yield jpeg.tobytes()
        return
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            ret, jpeg = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            yield jpeg.tobytes()
    finally:
        cap.release()


def main():
    # Start UDP receiver thread
    udp_thread = threading.Thread(target=udp_receiver_thread, args=(ROS_UDP_PORT, udp_data_queue), daemon=True)
    udp_thread.start()
    # Start status updater
    status_thread = threading.Thread(target=update_status, daemon=True)
    status_thread.start()
    # Start HTTP server
    server = HTTPServer((SERVER_HOST, SERVER_PORT), RobotHTTPRequestHandler)
    print(f"Driver HTTP server starting at http://{SERVER_HOST}:{SERVER_PORT}/")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    server.server_close()


if __name__ == "__main__":
    main()