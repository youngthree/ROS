import os
import io
import asyncio
import json
import socket
import struct
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse
from http import HTTPStatus

import cv2
import numpy as np

# ==== Environment Variables ====
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
ROS_UDP_PORT = int(os.environ.get("ROS_UDP_PORT", "9000"))
RTSP_PORT = int(os.environ.get("RTSP_PORT", "554"))
HTTP_HOST = os.environ.get("HTTP_HOST", "0.0.0.0")
HTTP_PORT = int(os.environ.get("HTTP_PORT", "8080"))

# ==== ROS/UDP Data Receiver ====
UDP_BUFFER_SIZE = 65536
latest_status = {}
udp_running = True

def udp_receiver():
    global latest_status
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((DEVICE_IP, ROS_UDP_PORT))
    while udp_running:
        try:
            data, _ = sock.recvfrom(UDP_BUFFER_SIZE)
            try:
                msg = json.loads(data.decode("utf-8"))
                latest_status.update(msg)
            except Exception:
                continue
        except Exception:
            break
    sock.close()

udp_thread = threading.Thread(target=udp_receiver, daemon=True)
udp_thread.start()

# ==== RTSP-to-MJPEG HTTP Proxy ====
class RTSP2MJPEGProxy:
    def __init__(self, rtsp_url):
        self.rtsp_url = rtsp_url
        self.capture = None
        self.lock = threading.Lock()
        self.running = False

    def start(self):
        with self.lock:
            if self.running:
                return
            self.capture = cv2.VideoCapture(self.rtsp_url)
            self.running = True

    def stop(self):
        with self.lock:
            if self.capture:
                self.capture.release()
                self.capture = None
            self.running = False

    def get_frame(self):
        with self.lock:
            if not self.running or not self.capture:
                return None
            ret, frame = self.capture.read()
            if not ret:
                return None
            _, jpeg = cv2.imencode('.jpg', frame)
            return jpeg.tobytes()

    def is_running(self):
        with self.lock:
            return self.running

# ==== HTTP Server ====
class JueyingHTTPRequestHandler(BaseHTTPRequestHandler):
    rtsp_url = f"rtsp://{DEVICE_IP}:{RTSP_PORT}/stream1"
    mjpeg_proxy = RTSP2MJPEGProxy(rtsp_url)

    # /status endpoint
    def do_GET(self):
        parsed_path = urlparse(self.path)
        if parsed_path.path == "/status":
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            status = dict(latest_status)
            self.wfile.write(json.dumps(status).encode("utf-8"))
        elif parsed_path.path == "/video":
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            self.mjpeg_proxy.start()
            try:
                while True:
                    frame = self.mjpeg_proxy.get_frame()
                    if frame is None:
                        continue
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
            except (ConnectionAbortedError, ConnectionResetError, BrokenPipeError):
                return
        else:
            self.send_error(HTTPStatus.NOT_FOUND, "Endpoint not found")

    # /task and /move endpoints
    def do_POST(self):
        parsed_path = urlparse(self.path)
        content_length = int(self.headers.get("Content-Length", 0))
        post_data = self.rfile.read(content_length)
        try:
            payload = json.loads(post_data.decode("utf-8"))
        except Exception:
            self.send_error(HTTPStatus.BAD_REQUEST, "Invalid JSON")
            return

        if parsed_path.path == "/move":
            # Simulate sending movement commands via UDP to robot (cmd_vel)
            try:
                msg = json.dumps(payload).encode("utf-8")
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                    s.sendto(msg, (DEVICE_IP, ROS_UDP_PORT))
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"result": "move command sent"}).encode("utf-8"))
            except Exception as e:
                self.send_error(HTTPStatus.INTERNAL_SERVER_ERROR, str(e))
        elif parsed_path.path == "/task":
            # Simulate script management (start/stop SLAM, LiDAR, etc.) via UDP
            try:
                msg = json.dumps({"task": payload}).encode("utf-8")
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                    s.sendto(msg, (DEVICE_IP, ROS_UDP_PORT))
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"result": "task command sent"}).encode("utf-8"))
            except Exception as e:
                self.send_error(HTTPStatus.INTERNAL_SERVER_ERROR, str(e))
        else:
            self.send_error(HTTPStatus.NOT_FOUND, "Endpoint not found")

    def log_message(self, format, *args):
        return  # Suppress logging

def run():
    server = HTTPServer((HTTP_HOST, HTTP_PORT), JueyingHTTPRequestHandler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        global udp_running
        udp_running = False
        JueyingHTTPRequestHandler.mjpeg_proxy.stop()
        server.server_close()

if __name__ == '__main__':
    run()