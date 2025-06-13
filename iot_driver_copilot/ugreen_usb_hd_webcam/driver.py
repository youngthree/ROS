import os
import io
import cv2
import time
import json
import threading
from flask import Flask, Response, jsonify, send_file

app = Flask(__name__)

# Configuration from environment variables
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", "8080"))
WEBCAM_INDEX = int(os.environ.get("WEBCAM_INDEX", "0"))  # USB webcam index
VIDEO_FPS = int(os.environ.get("VIDEO_FPS", "15"))

class WebcamStream:
    def __init__(self, index):
        self.index = index
        self.capture = None
        self.lock = threading.Lock()
        self.is_opened = False
        self.last_frame = None
        self.last_read_time = 0
        self.open()

    def open(self):
        with self.lock:
            if self.capture is None or not self.capture.isOpened():
                self.capture = cv2.VideoCapture(self.index)
                self.is_opened = self.capture.isOpened()

    def close(self):
        with self.lock:
            if self.capture is not None:
                self.capture.release()
                self.capture = None
                self.is_opened = False

    def read(self):
        with self.lock:
            if not self.capture or not self.capture.isOpened():
                self.open()
            if self.capture and self.capture.isOpened():
                ret, frame = self.capture.read()
                if ret:
                    self.last_frame = frame
                    self.last_read_time = time.time()
                    return frame
                else:
                    self.is_opened = False
            return None

    def get_status(self):
        with self.lock:
            status = {
                "device_name": "UGREEN USB HD Webcam",
                "device_model": "CM678",
                "manufacturer": "UGREEN",
                "device_type": "Webcam",
                "opened": self.capture.isOpened() if self.capture else False,
                "last_read": self.last_read_time,
                "video_fps": VIDEO_FPS,
                "webcam_index": self.index
            }
            return status

webcam = WebcamStream(WEBCAM_INDEX)

@app.route('/video', methods=['GET'])
def video_stream():
    def gen_frames():
        while True:
            frame = webcam.read()
            if frame is not None:
                ret, jpeg = cv2.imencode('.jpg', frame)
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
            else:
                # Wait and retry if frame is not available
                time.sleep(0.1)
            time.sleep(1.0 / VIDEO_FPS)
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/capture', methods=['GET'])
def capture_image():
    frame = webcam.read()
    if frame is None:
        return jsonify({"error": "Unable to capture image from webcam"}), 500
    ret, jpeg = cv2.imencode('.jpg', frame)
    if not ret:
        return jsonify({"error": "Failed to encode image"}), 500
    return Response(jpeg.tobytes(), mimetype='image/jpeg')

@app.route('/probe', methods=['GET'])
def probe_status():
    status = webcam.get_status()
    return jsonify(status)

def cleanup():
    webcam.close()

import atexit
atexit.register(cleanup)

if __name__ == "__main__":
    app.run(host=SERVER_HOST, port=SERVER_PORT, threaded=True)