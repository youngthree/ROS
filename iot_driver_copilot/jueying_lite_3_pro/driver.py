import os
import json
import threading
import socket
import struct
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs

# Configuration from environment variables
DEVICE_IP = os.environ.get('DEVICE_IP', '127.0.0.1')
ROS_UDP_PORT = int(os.environ.get('ROS_UDP_PORT', 15000))  # For demo UDP status
RTSP_URL = os.environ.get('RTSP_URL', f'rtsp://{DEVICE_IP}/live')  # RTSP stream
HTTP_SERVER_HOST = os.environ.get('HTTP_SERVER_HOST', '0.0.0.0')
HTTP_SERVER_PORT = int(os.environ.get('HTTP_SERVER_PORT', 8080))

# Dummy UDP status fetcher (simulate robot status data via UDP)
def fetch_udp_status(timeout=1.0):
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.settimeout(timeout)
    try:
        udp_sock.sendto(b'STATUS?', (DEVICE_IP, ROS_UDP_PORT))
        data, _ = udp_sock.recvfrom(4096)
        return json.loads(data)
    except Exception:
        # Simulate status if UDP not available
        return {
            "localization": {"x": 1.23, "y": 4.56, "theta": 0.78},
            "navigation": {"state": "idle", "goal": [2.0, 3.0]},
            "imu": {"ax": 0.0, "ay": 0.1, "az": 9.8},
            "battery": {"voltage": 24.5, "level": 87}
        }
    finally:
        udp_sock.close()

# RTSP to HTTP MJPEG streaming
import cv2
import queue

class RTSPProxy:
    def __init__(self, rtsp_url):
        self.rtsp_url = rtsp_url
        self.frame_queue = queue.Queue(maxsize=10)
        self.running = False

    def start(self):
        if self.running: return
        self.running = True
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join()

    def _worker(self):
        cap = cv2.VideoCapture(self.rtsp_url)
        while self.running:
            ret, frame = cap.read()
            if not ret:
                continue
            ret, jpeg = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            try:
                self.frame_queue.put(jpeg.tobytes(), timeout=1)
            except queue.Full:
                try:
                    self.frame_queue.get_nowait()  # Drop oldest
                except queue.Empty:
                    pass
        cap.release()

    def get_frame(self):
        try:
            return self.frame_queue.get(timeout=2)
        except queue.Empty:
            return None

rtsp_proxy = RTSPProxy(RTSP_URL)
rtsp_proxy.start()

# HTTP API Handlers
class JueyingHandler(BaseHTTPRequestHandler):
    def _set_headers(self, code=200, content_type="application/json"):
        self.send_response(code)
        self.send_header('Content-type', content_type)
        self.end_headers()

    def do_GET(self):
        if self.path == '/status':
            # Fetch status data via UDP (or simulate)
            status = fetch_udp_status()
            self._set_headers()
            self.wfile.write(json.dumps(status).encode('utf-8'))
        elif self.path.startswith('/video'):
            self._set_headers(200, 'multipart/x-mixed-replace; boundary=frame')
            self.stream_mjpeg()
        else:
            self._set_headers(404)
            self.wfile.write(b'{"error":"Not found"}')

    def stream_mjpeg(self):
        while True:
            frame = rtsp_proxy.get_frame()
            if frame is None:
                continue
            self.wfile.write(b"--frame\r\n")
            self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
            self.wfile.write(frame)
            self.wfile.write(b"\r\n")

    def do_POST(self):
        content_length = int(self.headers.get('Content-Length', 0))
        post_data = self.rfile.read(content_length)
        try:
            request_json = json.loads(post_data.decode('utf-8'))
        except Exception:
            self._set_headers(400)
            self.wfile.write(b'{"error":"Invalid JSON"}')
            return

        if self.path == '/task':
            # Manage operational scripts: start/stop SLAM, LiDAR, navigation (simulate)
            action = request_json.get('action')
            script_type = request_json.get('type')
            # Simulate result
            result = {"result": f"{action} {script_type} executed"}
            self._set_headers()
            self.wfile.write(json.dumps(result).encode('utf-8'))
        elif self.path == '/move':
            # Send movement commands (simulate publishing to ROS topic)
            velocity = request_json.get('velocity', {})
            # Simulate result
            result = {"result": "cmd_vel command accepted", "velocity": velocity}
            self._set_headers()
            self.wfile.write(json.dumps(result).encode('utf-8'))
        else:
            self._set_headers(404)
            self.wfile.write(b'{"error":"Not found"}')

def run_server():
    server = HTTPServer((HTTP_SERVER_HOST, HTTP_SERVER_PORT), JueyingHandler)
    print(f"Jueying Lite3 Pro HTTP API running on {HTTP_SERVER_HOST}:{HTTP_SERVER_PORT}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        rtsp_proxy.stop()
        server.server_close()

if __name__ == '__main__':
    run_server()