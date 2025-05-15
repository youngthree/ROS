import os
import sys
import io
import threading
import json
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs
import socket
import struct

# Configuration from environment
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
ROS_UDP_PORT = int(os.environ.get("ROS_UDP_PORT", "9000"))
RTSP_URL = os.environ.get("RTSP_URL", f"rtsp://{DEVICE_IP}/live")
HTTP_SERVER_HOST = os.environ.get("HTTP_SERVER_HOST", "0.0.0.0")
HTTP_SERVER_PORT = int(os.environ.get("HTTP_SERVER_PORT", "8080"))

# For demo: mock robot status data
robot_status = {
    "localization": {"x": 1.0, "y": 2.0, "theta": 0.5},
    "navigation": {"active": True, "goal": [5, 5]},
    "sensors": {"imu": {"yaw": 0.01, "pitch": 0.02, "roll": 0.03}},
    "timestamp": time.time()
}
robot_status_lock = threading.Lock()

# For demo: mock task state
task_state = {
    "slam": False,
    "lidar": False,
    "navigation": False
}
task_state_lock = threading.Lock()

# UDP ROS Message Listener (for status simulation)
def udp_ros_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((DEVICE_IP, ROS_UDP_PORT))
    except Exception as e:
        print(f"Could not bind UDP socket: {e}", file=sys.stderr)
        return
    while True:
        data, _ = sock.recvfrom(4096)
        # Parse fake data for status
        try:
            msg = json.loads(data.decode())
            with robot_status_lock:
                robot_status.update(msg)
                robot_status["timestamp"] = time.time()
        except Exception:
            continue

udp_thread = threading.Thread(target=udp_ros_listener, daemon=True)
udp_thread.start()

# RTSP-to-MJPEG proxy (raw, simple implementation)
def parse_h264_nal_units(data):
    """Find NAL units in H264 stream for demonstration.
    This does not decode H264, but splits NALs for example MJPEG conversion."""
    nals = []
    i = 0
    while True:
        start = data.find(b'\x00\x00\x00\x01', i)
        if start == -1:
            break
        next_start = data.find(b'\x00\x00\x00\x01', start + 4)
        if next_start == -1:
            nals.append(data[start:])
            break
        nals.append(data[start:next_start])
        i = next_start
    return nals

def h264_to_mjpeg_generator(rtsp_url):
    # NOTE: No external tools, so no ffmpeg/gst. This will not produce a real MJPEG,
    # but as a placeholder, we just yield "fake" JPEG frames for demonstration.
    # In a real implementation, you would use a built-in python H264 decoder,
    # but such does not exist in standard library.
    boundary = "--frame"
    while True:
        # Simulate a JPEG frame (black image)
        jpg = (
            b'\xff\xd8' + b'\xff' * 1000 + b'\xff\xd9'
        )  # not a real image, just for stream test
        yield (
            f"{boundary}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(jpg)}\r\n\r\n".encode()
            + jpg + b"\r\n"
        )
        time.sleep(0.1)

class RequestHandler(BaseHTTPRequestHandler):
    def _set_headers(self, status=200, content_type="application/json"):
        self.send_response(status)
        self.send_header('Content-type', content_type)
        self.end_headers()

    def do_GET(self):
        url_parts = urlparse(self.path)
        if url_parts.path == "/status":
            with robot_status_lock:
                data = json.dumps(robot_status).encode()
            self._set_headers(200, "application/json")
            self.wfile.write(data)
        elif url_parts.path == "/video":
            # MJPEG Stream
            self.send_response(200)
            self.send_header(
                'Content-type',
                'multipart/x-mixed-replace; boundary=--frame'
            )
            self.end_headers()
            try:
                for frame in h264_to_mjpeg_generator(RTSP_URL):
                    self.wfile.write(frame)
                    self.wfile.flush()
            except (ConnectionResetError, BrokenPipeError):
                pass
        else:
            self._set_headers(404)
            self.wfile.write(b'{"error":"Not Found"}')

    def do_POST(self):
        url_parts = urlparse(self.path)
        content_length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(content_length) if content_length > 0 else b""
        if url_parts.path == "/move":
            # Expect JSON: {"linear": {"x": float, "y": float}, "angular": {"z": float}}
            try:
                cmd = json.loads(body.decode())
                # In a real implementation, publish to ROS/UDP topic
                # For now, just log and return accepted
                self._set_headers(200)
                self.wfile.write(json.dumps({"result":"accepted", "command":cmd}).encode())
            except Exception as e:
                self._set_headers(400)
                self.wfile.write(json.dumps({"error":str(e)}).encode())
        elif url_parts.path == "/task":
            # Expect JSON: {"action": "start"/"stop", "script": "slam"/"lidar"/"navigation"}
            try:
                req = json.loads(body.decode())
                action = req.get("action")
                script = req.get("script")
                if action not in ["start", "stop"] or script not in task_state:
                    raise ValueError("Invalid action or script")
                with task_state_lock:
                    task_state[script] = (action == "start")
                self._set_headers(200)
                self.wfile.write(json.dumps({"result":f"{action}ed {script}"}).encode())
            except Exception as e:
                self._set_headers(400)
                self.wfile.write(json.dumps({"error":str(e)}).encode())
        else:
            self._set_headers(404)
            self.wfile.write(b'{"error":"Not Found"}')

def run_server():
    server_address = (HTTP_SERVER_HOST, HTTP_SERVER_PORT)
    httpd = HTTPServer(server_address, RequestHandler)
    print(f"Driver HTTP server running at http://{HTTP_SERVER_HOST}:{HTTP_SERVER_PORT}/")
    httpd.serve_forever()

if __name__ == "__main__":
    run_server()