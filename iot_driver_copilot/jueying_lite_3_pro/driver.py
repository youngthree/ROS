import os
import json
import socket
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

# Environment variables for device and server configuration
DEVICE_IP = os.environ.get('DEVICE_IP', '127.0.0.1')
DEVICE_UDP_PORT = int(os.environ.get('DEVICE_UDP_PORT', '9000'))
SERVER_HOST = os.environ.get('SERVER_HOST', '0.0.0.0')
SERVER_PORT = int(os.environ.get('SERVER_PORT', '8000'))

# UDP socket setup for sending commands and receiving data
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_sock.settimeout(2.0)

# In-memory latest data cache (as would be streamed from UDP subscriptions)
status_data = {
    "leg_odom": {},
    "imu_data": {},
    "joint_states": {},
    "handle_state": {},
    "ultrasound_distance": {},
    "cmd_vel": {},
    "cmd_vel_corrected": {}
}

# A minimal UDP listener thread to receive status updates
def udp_listener():
    listen_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listen_sock.bind((SERVER_HOST, DEVICE_UDP_PORT))
    while True:
        try:
            data, addr = listen_sock.recvfrom(8192)
            try:
                msg = json.loads(data.decode())
                # We expect JSON messages with a 'topic' and 'payload'
                topic = msg.get('topic')
                payload = msg.get('payload')
                if topic and topic in status_data:
                    status_data[topic] = payload
            except Exception:
                continue
        except Exception:
            continue

listener_thread = threading.Thread(target=udp_listener, daemon=True)
listener_thread.start()

# Helper: Send command to robot over UDP
def send_command(cmd, extra=None):
    packet = {
        "cmd": cmd
    }
    if extra:
        packet.update(extra)
    udp_sock.sendto(json.dumps(packet).encode(), (DEVICE_IP, DEVICE_UDP_PORT))
    # Optionally, wait for ACK or response (not implemented for simplicity)

# HTTP Handler
class RobotHTTPRequestHandler(BaseHTTPRequestHandler):
    def _send_json(self, data, code=200):
        resp = json.dumps(data).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(resp)))
        self.end_headers()
        self.wfile.write(resp)

    def do_GET(self):
        if self.path == '/status':
            # Return the latest cached status data
            self._send_json({"status": status_data})
        else:
            self.send_error(404, "Not found")

    def do_POST(self):
        command_map = {
            "/move/forward": "move/forward",
            "/move/backward": "move/backward",
            "/move/left": "move/left",
            "/move/right": "move/right",
            "/move/stop": "move/stop",
            "/move/stop_all": "move/stop_all",
            "/move/rotate_left": "move/rotate_left",
            "/move/rotate right": "move/rotate_right",
            "/stand": "stand",
            "/greet": "greet",
            "/jump": "jump",
            "/twist": "twist",
            "/emerg": "emergency_stop"
        }
        path = self.path
        cmd = command_map.get(path)
        if cmd:
            try:
                content_length = int(self.headers.get('Content-Length', 0))
            except (ValueError, TypeError):
                content_length = 0
            extra = None
            if content_length > 0:
                try:
                    body = self.rfile.read(content_length)
                    extra = json.loads(body.decode())
                except Exception:
                    extra = None
            send_command(cmd, extra)
            self._send_json({"result": "ok", "command": cmd})
        else:
            self.send_error(404, "Not found")

def run_server():
    httpd = HTTPServer((SERVER_HOST, SERVER_PORT), RobotHTTPRequestHandler)
    print(f"HTTP server running at http://{SERVER_HOST}:{SERVER_PORT}/")
    httpd.serve_forever()

if __name__ == '__main__':
    run_server()