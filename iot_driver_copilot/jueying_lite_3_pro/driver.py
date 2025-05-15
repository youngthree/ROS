import os
import io
import json
import threading
import time
from flask import Flask, request, Response, jsonify, stream_with_context
import requests
import cv2
import numpy as np

# Environment variable reading
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
RTSP_PORT = int(os.environ.get("RTSP_PORT", "554"))
RTSP_PATH = os.environ.get("RTSP_PATH", "/stream1")
HTTP_HOST = os.environ.get("HTTP_HOST", "0.0.0.0")
HTTP_PORT = int(os.environ.get("HTTP_PORT", "8080"))
ROS_STATUS_ENDPOINT = os.environ.get("ROS_STATUS_ENDPOINT", f"http://{DEVICE_IP}:5000/status")
ROS_MOVE_ENDPOINT = os.environ.get("ROS_MOVE_ENDPOINT", f"http://{DEVICE_IP}:5000/move")
ROS_TASK_ENDPOINT = os.environ.get("ROS_TASK_ENDPOINT", f"http://{DEVICE_IP}:5000/task")

app = Flask(__name__)

@app.route('/video', methods=['GET'])
def video_feed():
    def generate():
        rtsp_url = f"rtsp://{DEVICE_IP}:{RTSP_PORT}{RTSP_PATH}"
        cap = cv2.VideoCapture(rtsp_url)
        if not cap.isOpened():
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + b"\r\n"
            return
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                ret, jpeg = cv2.imencode('.jpg', frame)
                if not ret:
                    continue
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
        finally:
            cap.release()
    return Response(stream_with_context(generate()), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/status', methods=['GET'])
def get_status():
    # Proxy the status request to the ROS endpoint
    try:
        r = requests.get(ROS_STATUS_ENDPOINT, timeout=3)
        return Response(r.content, status=r.status_code, mimetype='application/json')
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/move', methods=['POST'])
def move():
    # Proxy the move command to the ROS endpoint
    try:
        data = request.get_json(force=True)
        r = requests.post(ROS_MOVE_ENDPOINT, json=data, timeout=3)
        return Response(r.content, status=r.status_code, mimetype='application/json')
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/task', methods=['POST'])
def task():
    # Proxy the task command to the ROS endpoint
    try:
        data = request.get_json(force=True)
        r = requests.post(ROS_TASK_ENDPOINT, json=data, timeout=3)
        return Response(r.content, status=r.status_code, mimetype='application/json')
    except Exception as e:
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    app.run(host=HTTP_HOST, port=HTTP_PORT, threaded=True)