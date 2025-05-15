import os
import threading
import io
import json
import time
import socket
import struct
import base64

from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs

# Environment variable configuration
DEVICE_IP = os.environ.get('DEVICE_IP', '127.0.0.1')
DEVICE_ROS_UDP_PORT = int(os.environ.get('DEVICE_ROS_UDP_PORT', '15003'))
DEVICE_RTSP_URL = os.environ.get('DEVICE_RTSP_URL', f'rtsp://{DEVICE_IP}:8554/live')
HTTP_SERVER_HOST = os.environ.get('HTTP_SERVER_HOST', '0.0.0.0')
HTTP_SERVER_PORT = int(os.environ.get('HTTP_SERVER_PORT', '8080'))

# For demonstration, we use UDP for status data and movement commands.
# In real settings, you would use ROS libraries, but only sockets and stdlib are allowed.

# Simple UDP helper for sending commands and receiving status
def send_udp_command(command_dict):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        message = json.dumps(command_dict).encode('utf-8')
        sock.sendto(message, (DEVICE_IP, DEVICE_ROS_UDP_PORT))
        sock.settimeout(1.0)
        try:
            data, _ = sock.recvfrom(65536)
            return json.loads(data.decode('utf-8'))
        except socket.timeout:
            return {"success": False, "error": "No response from device"}
    finally:
        sock.close()

# A simple RTSP->MJPEG over HTTP proxy
import cv2
import numpy as np

class StreamProxy(threading.Thread):
    def __init__(self, rtsp_url):
        super().__init__()
        self.rtsp_url = rtsp_url
        self.frame = None
        self.running = False
        self.lock = threading.Lock()

    def run(self):
        self.running = True
        cap = cv2.VideoCapture(self.rtsp_url)
        if not cap.isOpened():
            self.running = False
            return
        while self.running:
            ret, frame = cap.read()
            if not ret:
                continue
            with self.lock:
                self.frame = frame
        cap.release()

    def get_frame(self):
        with self.lock:
            if self.frame is None:
                return None
            ret, jpeg = cv2.imencode('.jpg', self.frame)
            if not ret:
                return None
            return jpeg.tobytes()

    def stop(self):
        self.running = False

stream_proxy = StreamProxy(DEVICE_RTSP_URL)
stream_proxy.daemon = True
stream_proxy.start()

class DriverRequestHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        parsed_path = urlparse(self.path)
        if parsed_path.path == '/task':
            self.handle_task()
        elif parsed_path.path == '/move':
            self.handle_move()
        else:
            self.send_error(404, 'Not Found')

    def do_GET(self):
        parsed_path = urlparse(self.path)
        if parsed_path.path == '/status':
            self.handle_status()
        elif parsed_path.path == '/video':
            self.handle_video()
        else:
            self.send_error(404, 'Not Found')

    def handle_task(self):
        content_length = int(self.headers.get('Content-Length', 0))
        raw_body = self.rfile.read(content_length)
        try:
            body = json.loads(raw_body.decode('utf-8'))
        except Exception:
            self.send_error(400, 'Invalid JSON')
            return
        # Translate to UDP command for the robot (action, script_type)
        command = {
            'type': 'task',
            'action': body.get('action'),
            'script_type': body.get('script_type')
        }
        result = send_udp_command(command)
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(result).encode('utf-8'))

    def handle_move(self):
        content_length = int(self.headers.get('Content-Length', 0))
        raw_body = self.rfile.read(content_length)
        try:
            body = json.loads(raw_body.decode('utf-8'))
        except Exception:
            self.send_error(400, 'Invalid JSON')
            return
        # Translate to UDP command for the robot (velocity commands)
        command = {
            'type': 'move',
            'cmd_vel': body.get('cmd_vel'),
            'cmd_vel_corrected': body.get('cmd_vel_corrected')
        }
        result = send_udp_command(command)
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(result).encode('utf-8'))

    def handle_status(self):
        # Ask the robot for status data via UDP
        command = {'type': 'status'}
        result = send_udp_command(command)
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(result).encode('utf-8'))

    def handle_video(self):
        self.send_response(200)
        self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        try:
            while True:
                frame = stream_proxy.get_frame()
                if frame is None:
                    time.sleep(0.1)
                    continue
                self.wfile.write(b'--frame\r\n')
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(frame)))
                self.end_headers()
                self.wfile.write(frame)
                self.wfile.write(b'\r\n')
                time.sleep(0.04)  # ~25fps
        except Exception:
            pass  # Client disconnected or error

    def log_message(self, format, *args):
        # Suppress default logging
        return

def run():
    server = HTTPServer((HTTP_SERVER_HOST, HTTP_SERVER_PORT), DriverRequestHandler)
    print(f"HTTP server running at http://{HTTP_SERVER_HOST}:{HTTP_SERVER_PORT}/")
    try:
        server.serve_forever()
    finally:
        stream_proxy.stop()

if __name__ == '__main__':
    run()