import os
import io
import json
import asyncio
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs
import socket
import struct

try:
    import cv2
except ImportError:
    cv2 = None

# ==== Configuration from Environment Variables ====
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
ROS_UDP_PORT = int(os.environ.get("ROS_UDP_PORT", "15000"))
RTSP_URL = os.environ.get("RTSP_URL", f"rtsp://{DEVICE_IP}:8554/live")
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", "8080"))

# ==== In-memory Robot State ====
robot_status = {
    "localization": {},
    "navigation": {},
    "sensors": {},
    "operational": {}
}

# ==== UDP Listener for Status (ROS/UDP) ====
def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((DEVICE_IP, ROS_UDP_PORT))
    while True:
        data, _ = sock.recvfrom(65535)
        try:
            msg = json.loads(data.decode(errors='ignore'))
            # Update status with received data (basic merge)
            robot_status["sensors"].update(msg)
        except Exception:
            pass

def start_udp_listener():
    t = threading.Thread(target=udp_listener, daemon=True)
    t.start()

# ==== HTTP Handler ====
class IoTDriverHandler(BaseHTTPRequestHandler):
    server_version = "JueyingLite3ProHTTP/1.0"

    def _set_headers(self, code=200, content_type="application/json"):
        self.send_response(code)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Content-type', content_type)
        self.end_headers()

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'POST, GET, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/status":
            self._set_headers()
            self.wfile.write(json.dumps(robot_status).encode())
        elif parsed.path == "/video":
            if cv2 is None:
                self._set_headers(500)
                self.wfile.write(b'{"error": "OpenCV is not installed"}')
                return
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            try:
                for frame in rtsp_to_mjpeg_generator():
                    self.wfile.write(frame)
            except Exception:
                pass
        else:
            self._set_headers(404)
            self.wfile.write(b'{"error": "Not found"}')

    def do_POST(self):
        parsed = urlparse(self.path)
        content_length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(content_length)
        if parsed.path == "/move":
            try:
                cmd = json.loads(body.decode())
                # In real scenario, send this to the robot via ROS/UDP/etc.
                robot_status["operational"]["last_move"] = cmd
                self._set_headers()
                self.wfile.write(b'{"status":"ok"}')
            except Exception as e:
                self._set_headers(400)
                self.wfile.write(json.dumps({"error": str(e)}).encode())
        elif parsed.path == "/task":
            try:
                task = json.loads(body.decode())
                # Emulate task operation, update status
                action = task.get("action")
                script_type = task.get("script_type")
                robot_status["operational"]["last_task"] = {
                    "action": action,
                    "script_type": script_type
                }
                self._set_headers()
                self.wfile.write(b'{"status":"ok"}')
            except Exception as e:
                self._set_headers(400)
                self.wfile.write(json.dumps({"error": str(e)}).encode())
        else:
            self._set_headers(404)
            self.wfile.write(b'{"error": "Not found"}')

# ==== RTSP to MJPEG HTTP Streaming ====
def rtsp_to_mjpeg_generator():
    cap = cv2.VideoCapture(RTSP_URL)
    if not cap.isOpened():
        yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + b'' + b'\r\n'
        return
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        ret, jpeg = cv2.imencode('.jpg', frame)
        if not ret:
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' +
               jpeg.tobytes() + b'\r\n')
    cap.release()

# ==== Main Entrypoint ====
def run_server():
    start_udp_listener()
    httpd = HTTPServer((SERVER_HOST, SERVER_PORT), IoTDriverHandler)
    print(f"Serving on http://{SERVER_HOST}:{SERVER_PORT}")
    httpd.serve_forever()

if __name__ == "__main__":
    run_server()