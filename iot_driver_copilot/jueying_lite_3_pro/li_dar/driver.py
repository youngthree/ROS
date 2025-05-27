import os
import socket
import struct
import threading
import json
from http.server import BaseHTTPRequestHandler, HTTPServer

# Configuration from environment variables
ROBOT_IP = os.environ.get('ROBOT_IP', '192.168.123.10')
ROBOT_UDP_PORT = int(os.environ.get('ROBOT_UDP_PORT', '8001'))  # UDP port for commands/status
HTTP_HOST = os.environ.get('HTTP_HOST', '0.0.0.0')
HTTP_PORT = int(os.environ.get('HTTP_PORT', '8080'))

# UDP socket setup (thread-safe)
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_sock.settimeout(2)

# Robot command mapping
COMMANDS = {
    "move/forward": b'CMD_FORWARD',
    "move/backward": b'CMD_BACKWARD',
    "move/left": b'CMD_LEFT',
    "move/right": b'CMD_RIGHT',
    "move/stop": b'CMD_STOP',
    "move/stop_all": b'CMD_STOP_ALL',
    "move/rotate_left": b'CMD_ROTATE_LEFT',
    "move/rotate_right": b'CMD_ROTATE_RIGHT',
    "stand": b'CMD_STAND',
    "hello": b'CMD_GREET',
    "jump": b'CMD_JUMP',
    "twist": b'CMD_TWIST',
    "e_stop": b'CMD_ESTOP'
}

STATUS_REQUEST = b'REQ_STATUS'

def send_udp_command(cmd_bytes):
    try:
        udp_sock.sendto(cmd_bytes, (ROBOT_IP, ROBOT_UDP_PORT))
        # For commands, we can just return OK
        return True, None
    except Exception as e:
        return False, str(e)

def request_status():
    try:
        udp_sock.sendto(STATUS_REQUEST, (ROBOT_IP, ROBOT_UDP_PORT))
        data, _ = udp_sock.recvfrom(4096)
        # Try to decode as JSON, else fallback to raw
        try:
            status = json.loads(data.decode('utf-8'))
        except Exception:
            status = {"raw_status": data.hex()}
        return status
    except Exception as e:
        return {"error": str(e)}

class RobotDriverHandler(BaseHTTPRequestHandler):
    def _set_headers(self, code=200, content_type='application/json'):
        self.send_response(code)
        self.send_header('Content-type', content_type)
        self.end_headers()

    def _json_response(self, payload, code=200):
        self._set_headers(code)
        self.wfile.write(json.dumps(payload).encode('utf-8'))

    def do_GET(self):
        if self.path == "/status":
            status = request_status()
            self._json_response(status)
        else:
            self._json_response({"error": "not found"}, code=404)

    def do_POST(self):
        path = self.path.rstrip('/')
        action_map = {
            '/move/forward': "move/forward",
            '/move/backward': "move/backward",
            '/move/left': "move/left",
            '/move/right': "move/right",
            '/move/stop': "move/stop",
            '/move/stop_all': "move/stop_all",
            '/move/rotate_left': "move/rotate_left",
            '/move/rotate_right': "move/rotate_right",
            '/stand': "stand",
            '/hello': "hello",
            '/jump': "jump",
            '/twist': "twist",
            '/e_stop': "e_stop",
            '/greet': "hello",
        }
        if path in action_map:
            cmd_key = action_map[path]
            cmd_bytes = COMMANDS.get(cmd_key)
            if not cmd_bytes:
                self._json_response({'error': 'Command not implemented'}, code=501)
                return
            ok, err = send_udp_command(cmd_bytes)
            if ok:
                self._json_response({'result': 'ok', 'command': cmd_key})
            else:
                self._json_response({'error': err}, code=500)
        else:
            self._json_response({'error': 'not found'}, code=404)

    def log_message(self, format, *args):
        # Suppress logging, or redirect as needed
        return

def run_server():
    server = HTTPServer((HTTP_HOST, HTTP_PORT), RobotDriverHandler)
    print(f"Robot driver HTTP server running at http://{HTTP_HOST}:{HTTP_PORT}")
    server.serve_forever()

if __name__ == "__main__":
    run_server()