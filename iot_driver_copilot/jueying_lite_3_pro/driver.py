import os
import io
import cv2
import json
import time
import queue
import threading
import numpy as np
from flask import Flask, Response, request, jsonify, stream_with_context

# Environment variables
DEVICE_IP = os.environ.get('DEVICE_IP')
RTSP_PORT = int(os.environ.get('RTSP_PORT', 554))
RTSP_PATH = os.environ.get('RTSP_PATH', 'live')  # e.g. 'live' or 'ch1/stream1'
SERVER_HOST = os.environ.get('SERVER_HOST', '0.0.0.0')
SERVER_PORT = int(os.environ.get('SERVER_PORT', 8080))

# ROS/UDP config (if any future expansion)
ROS_MASTER_URI = os.environ.get('ROS_MASTER_URI')
UDP_PORT = os.environ.get('UDP_PORT')

# RTSP Stream URL
RTSP_URL = f"rtsp://{DEVICE_IP}:{RTSP_PORT}/{RTSP_PATH}"

app = Flask(__name__)

# Thread-safe queue for sharing frames between threads
frame_queue = queue.Queue(maxsize=10)
streaming_active = threading.Event()

def video_capture_worker():
    cap = cv2.VideoCapture(RTSP_URL)
    if not cap.isOpened():
        streaming_active.clear()
        return
    streaming_active.set()
    while streaming_active.is_set():
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.1)
            continue
        # Convert to JPEG
        ret, jpeg = cv2.imencode('.jpg', frame)
        if not ret:
            continue
        try:
            frame_queue.put(jpeg.tobytes(), timeout=1)
        except queue.Full:
            pass  # Drop frame if queue is full
    cap.release()

def gen_mjpeg_stream():
    global streaming_active
    # Start worker if not already running
    if not streaming_active.is_set():
        streaming_active.set()
        threading.Thread(target=video_capture_worker, daemon=True).start()
    while streaming_active.is_set():
        try:
            frame = frame_queue.get(timeout=5)
        except queue.Empty:
            break
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    streaming_active.clear()

@app.route('/video')
def video_feed():
    # HTTP MJPEG video stream
    return Response(gen_mjpeg_stream(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/task', methods=['POST'])
def manage_task():
    # Expected JSON: {"action": "start"|"stop", "script_type": "SLAM"|"LiDAR"|"navigation"}
    data = request.get_json(force=True)
    action = data.get("action")
    script_type = data.get("script_type")
    # Simulate script management (no shell execution)
    # In real deployments, interface with process control library or robot API
    if action not in ("start", "stop") or script_type not in ("SLAM", "LiDAR", "navigation"):
        return jsonify({'result': 'error', 'reason': 'Invalid action or script_type'}), 400
    # Fake operation status
    return jsonify({'result': 'success', 'action': action, 'script_type': script_type})

@app.route('/status', methods=['GET'])
def status():
    # Simulated status (replace with ROS/UDP/client calls as needed)
    status_data = {
        "localization": {"x": 1.23, "y": 2.34, "theta": 0.56},
        "navigation_state": "IDLE",
        "battery": 87,
        "imu": {"accel_x": 0.0, "accel_y": 0.0, "accel_z": 9.8},
        "ultrasound_distance": [0.5, 0.6, 0.7],
        "timestamp": time.time()
    }
    return jsonify(status_data)

@app.route('/move', methods=['POST'])
def move():
    # Expected JSON: {"linear": {"x": 0.1, "y":0, "z":0}, "angular": {"x":0, "y":0, "z":0.2}}
    command = request.get_json(force=True)
    # Fake command acceptance (should publish to ROS/UDP topic)
    # Here just echo the command back for demo
    return jsonify({'result': 'success', 'command': command})

@app.route('/')
def index():
    return '''
    <h1>Jueying Lite3 Pro Driver</h1>
    <ul>
        <li>MJPEG Video Stream: <a href="/video">/video</a></li>
        <li>POST /task (manage SLAM/LiDAR/Navigation)</li>
        <li>POST /move (send movement command)</li>
        <li>GET /status (fetch status)</li>
    </ul>
    '''

if __name__ == '__main__':
    app.run(host=SERVER_HOST, port=SERVER_PORT, debug=False, threaded=True)