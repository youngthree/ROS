import os
import json
import socket
from flask import Flask, request, jsonify

# Read configuration from environment variables
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
DEVICE_UDP_PORT = int(os.environ.get("DEVICE_UDP_PORT", "6000"))
HTTP_HOST = os.environ.get("HTTP_HOST", "0.0.0.0")
HTTP_PORT = int(os.environ.get("HTTP_PORT", "8080"))
UDP_TIMEOUT = float(os.environ.get("DEVICE_UDP_TIMEOUT", "2.0"))

# UDP message formats for each command
COMMAND_MAP = {
    "/move/forward": {
        "cmd": "move_forward",
        "payload_keys": ["speed", "duration"]
    },
    "/move/backward": {
        "cmd": "move_backward",
        "payload_keys": ["speed", "duration"]
    },
    "/turn/left": {
        "cmd": "turn_left",
        "payload_keys": ["angle", "speed"]
    },
    "/turn/right": {
        "cmd": "turn_right",
        "payload_keys": ["angle", "speed"]
    },
    "/stop": {
        "cmd": "stop",
        "payload_keys": []
    }
}

def send_udp_command(command: str, payload: dict = None):
    """
    Send a command via UDP to the robot and wait for a response.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(UDP_TIMEOUT)
    try:
        # Compose the message as JSON
        message = {"command": command}
        if payload:
            message.update(payload)
        msg_bytes = json.dumps(message).encode("utf-8")
        sock.sendto(msg_bytes, (DEVICE_IP, DEVICE_UDP_PORT))
        resp, _ = sock.recvfrom(4096)
        try:
            return json.loads(resp.decode("utf-8"))
        except Exception:
            return {"status": "ok", "raw_response": resp.decode("utf-8", errors="ignore")}
    except socket.timeout:
        return {"status": "error", "error": "No response from device"}
    except Exception as e:
        return {"status": "error", "error": str(e)}
    finally:
        sock.close()

app = Flask(__name__)

@app.route("/move/forward", methods=["POST"])
def move_forward():
    payload = request.get_json(silent=True) or {}
    send_payload = {k: payload[k] for k in COMMAND_MAP["/move/forward"]["payload_keys"] if k in payload}
    result = send_udp_command(COMMAND_MAP["/move/forward"]["cmd"], send_payload)
    return jsonify(result)

@app.route("/move/backward", methods=["POST"])
def move_backward():
    payload = request.get_json(silent=True) or {}
    send_payload = {k: payload[k] for k in COMMAND_MAP["/move/backward"]["payload_keys"] if k in payload}
    result = send_udp_command(COMMAND_MAP["/move/backward"]["cmd"], send_payload)
    return jsonify(result)

@app.route("/turn/left", methods=["POST"])
def turn_left():
    payload = request.get_json(silent=True) or {}
    send_payload = {k: payload[k] for k in COMMAND_MAP["/turn/left"]["payload_keys"] if k in payload}
    result = send_udp_command(COMMAND_MAP["/turn/left"]["cmd"], send_payload)
    return jsonify(result)

@app.route("/turn/right", methods=["POST"])
def turn_right():
    payload = request.get_json(silent=True) or {}
    send_payload = {k: payload[k] for k in COMMAND_MAP["/turn/right"]["payload_keys"] if k in payload}
    result = send_udp_command(COMMAND_MAP["/turn/right"]["cmd"], send_payload)
    return jsonify(result)

@app.route("/stop", methods=["POST"])
def stop():
    result = send_udp_command(COMMAND_MAP["/stop"]["cmd"])
    return jsonify(result)

if __name__ == "__main__":
    app.run(host=HTTP_HOST, port=HTTP_PORT)