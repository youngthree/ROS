import os
import io
import cv2
import time
import json
import queue
import base64
import socket
import struct
import threading
from flask import Flask, Response, request, jsonify, stream_with_context

# --- Environment Variables ---
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
RTSP_URL = os.environ.get("RTSP_URL", f"rtsp://{DEVICE_IP}:8554/live")
ROS_UDP_PORT = int(os.environ.get("ROS_UDP_PORT", "5555"))
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", "8080"))

# --- HTTP Server ---
app = Flask(__name__)

# -------- Video Stream (RTSP -> HTTP MJPEG) --------
def generate_mjpeg(rtsp_url):
    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + b'' + b'\r\n')
        return
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            _, jpeg = cv2.imencode('.jpg', frame)
            frame_bytes = jpeg.tobytes()
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
    finally:
        cap.release()

@app.route('/video')
def video_feed():
    return Response(
        stream_with_context(generate_mjpeg(RTSP_URL)),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

# -------- UDP/ROS Bridge for Status --------
status_data = {}
udp_thread_running = threading.Event()
udp_thread_running.set()

def udp_listener(status_data, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', port))
    sock.settimeout(1.0)
    while udp_thread_running.is_set():
        try:
            data, _ = sock.recvfrom(65535)
            try:
                msg = json.loads(data.decode('utf-8'))
                status_data.clear()
                status_data.update(msg)
            except Exception:
                continue
        except socket.timeout:
            continue
    sock.close()

udp_thread = threading.Thread(target=udp_listener, args=(status_data, ROS_UDP_PORT), daemon=True)
udp_thread.start()

@app.route('/status', methods=['GET'])
def get_status():
    # Return a basic status summary
    return jsonify({
        "status": "ok",
        "data": status_data
    })

# -------- POST /move: Send velocity commands (cmd_vel) --------
def send_udp_command(payload, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    data = json.dumps(payload).encode('utf-8')
    sock.sendto(data, (DEVICE_IP, port))
    sock.close()

@app.route('/move', methods=['POST'])
def move():
    payload = request.get_json(force=True)
    # Send as UDP for demonstration (normally via ROS topic)
    send_udp_command({"cmd": "move", "payload": payload}, ROS_UDP_PORT)
    return jsonify({"status": "sent", "payload": payload})

# -------- POST /task: Manage operational scripts --------
@app.route('/task', methods=['POST'])
def task():
    req = request.get_json(force=True)
    action = req.get("action", "").lower()
    script_type = req.get("script_type", "").lower()
    # Simulation: send to UDP or update status (no shell exec)
    send_udp_command({"cmd": "task", "action": action, "script_type": script_type}, ROS_UDP_PORT)
    return jsonify({"status": "ack", "action": action, "script_type": script_type})

# -------- Main --------
def shutdown_server():
    udp_thread_running.clear()
    udp_thread.join()

if __name__ == "__main__":
    try:
        app.run(host=SERVER_HOST, port=SERVER_PORT, threaded=True)
    finally:
        shutdown_server()