import os
import io
import threading
import json
import base64
import socket
import struct
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs

try:
    import cv2
except ImportError:
    cv2 = None

# Environment Variables
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
ROS_UDP_PORT = int(os.environ.get("ROS_UDP_PORT", "9000"))  # For UDP telemetry/status
RTSP_URL = os.environ.get("RTSP_URL", f"rtsp://{DEVICE_IP}:8554/live")  # RTSP stream url
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", "8080"))

# Internal State
status_data = {
    "localization": {},
    "navigation": {},
    "sensor": {},
    "operational": {}
}
task_state = {}

def udp_status_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((DEVICE_IP, ROS_UDP_PORT))
    while True:
        data, _ = sock.recvfrom(4096)
        try:
            msg = json.loads(data.decode())
            # Merge new data into status_data, could be customized as per protocol
            status_data.update(msg)
        except Exception:
            continue

def start_udp_listener():
    t = threading.Thread(target=udp_status_listener, daemon=True)
    t.start()

class MJPEGStreamHandler:
    def __init__(self, rtsp_url):
        self.rtsp_url = rtsp_url
        self.cap = None
        self.running = False

    def start(self):
        if cv2 is None:
            raise RuntimeError("OpenCV is required for RTSP streaming")
        self.cap = cv2.VideoCapture(self.rtsp_url)
        if not self.cap.isOpened():
            raise RuntimeError("Cannot open RTSP stream")
        self.running = True

    def stop(self):
        self.running = False
        if self.cap:
            self.cap.release()

    def frames(self):
        if not self.cap:
            self.start()
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                continue
            ret, jpeg = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            yield jpeg.tobytes()

stream_handler = MJPEGStreamHandler(RTSP_URL)

class DriverHandler(BaseHTTPRequestHandler):
    def _json_response(self, data, code=200):
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def _bad_request(self, msg='Bad request'):
        self._json_response({"error": msg}, code=400)

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/status":
            self._json_response(status_data)
        elif parsed.path == "/video":
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                stream_handler.running = True
                for frame in stream_handler.frames():
                    self.wfile.write(
                        b'--frame\r\n'
                        b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n'
                    )
            except Exception:
                pass
            finally:
                stream_handler.running = False
        else:
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b'404 Not Found')

    def do_POST(self):
        parsed = urlparse(self.path)
        length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(length)
        try:
            payload = json.loads(body.decode())
        except Exception:
            return self._bad_request("Invalid JSON payload")
        if parsed.path == "/task":
            action = payload.get("action")
            script = payload.get("script")
            if action not in ("start", "stop") or not script:
                return self._bad_request("Specify action=start|stop and script")
            # Simulate script control (would interface with robot controller here)
            task_state[script] = action
            self._json_response({"result": f"{action} {script} OK"})
        elif parsed.path == "/move":
            cmd = payload.get("cmd_vel") or payload.get("cmd_vel_corrected")
            if not cmd:
                return self._bad_request("Missing movement command")
            # Simulate sending movement command to robot (normally via ROS)
            # For demo: update status_data
            status_data["last_cmd"] = cmd
            self._json_response({"result": "Move command accepted"})
        else:
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b'404 Not Found')

def run():
    if cv2 is None:
        print("Error: OpenCV (cv2) is required for MJPEG streaming from RTSP.")
        exit(1)
    start_udp_listener()
    server = HTTPServer((SERVER_HOST, SERVER_PORT), DriverHandler)
    print(f"Jueying Lite3 Pro Driver HTTP server running on {SERVER_HOST}:{SERVER_PORT}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stream_handler.stop()
        server.server_close()

if __name__ == "__main__":
    run()