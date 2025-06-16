import os
import cv2
import threading
import time
import io
from flask import Flask, Response, jsonify, send_file

# Load configuration from environment variables
DEVICE_INDEX = int(os.environ.get('WEBCAM_DEVICE_INDEX', '0'))
SERVER_HOST = os.environ.get('SERVER_HOST', '0.0.0.0')
SERVER_PORT = int(os.environ.get('SERVER_PORT', '8080'))

app = Flask(__name__)

# Shared capture object and lock for thread safety
class Webcam:
    def __init__(self, device_index):
        self.device_index = device_index
        self.cap = None
        self.lock = threading.Lock()
        self.open_capture()

    def open_capture(self):
        with self.lock:
            if self.cap is None or not self.cap.isOpened():
                self.cap = cv2.VideoCapture(self.device_index, cv2.CAP_V4L2)
                # Try fallback if V4L2 is not available
                if not self.cap.isOpened():
                    self.cap = cv2.VideoCapture(self.device_index)
                # Set default resolution if needed
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    def is_opened(self):
        with self.lock:
            return self.cap is not None and self.cap.isOpened()

    def read(self):
        with self.lock:
            if not self.is_opened():
                self.open_capture()
            ret, frame = self.cap.read()
            return ret, frame

    def release(self):
        with self.lock:
            if self.cap is not None:
                self.cap.release()
                self.cap = None

webcam = Webcam(DEVICE_INDEX)

@app.route('/video')
def video_stream():
    def gen_frames():
        while True:
            ret, frame = webcam.read()
            if not ret:
                # Send blank frame if unable to capture
                time.sleep(0.05)
                continue
            # Encode frame as JPEG
            ret2, buffer = cv2.imencode('.jpg', frame)
            if not ret2:
                continue
            jpg_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpg_bytes + b'\r\n')
            time.sleep(0.04)  # ~25 FPS
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/capture')
def capture_image():
    ret, frame = webcam.read()
    if not ret:
        return jsonify({'success': False, 'message': 'Failed to capture image from webcam.'}), 500
    ret2, buffer = cv2.imencode('.jpg', frame)
    if not ret2:
        return jsonify({'success': False, 'message': 'Failed to encode image.'}), 500
    return send_file(
        io.BytesIO(buffer.tobytes()),
        mimetype='image/jpeg',
        as_attachment=False,
        download_name='capture.jpg'
    )

@app.route('/probe')
def probe_status():
    info = {
        "device_name": "UGREEN USB HD Webcam",
        "device_model": "CM678",
        "manufacturer": "UGREEN",
        "device_type": "Webcam",
        "device_index": DEVICE_INDEX,
        "status": "online" if webcam.is_opened() else "offline",
    }
    # Try to grab a frame and get its size
    ret, frame = webcam.read()
    if ret and frame is not None:
        info['last_frame_shape'] = {
            "height": int(frame.shape[0]),
            "width": int(frame.shape[1]),
            "channels": int(frame.shape[2]) if len(frame.shape) > 2 else 1
        }
    else:
        info['last_frame_shape'] = None
    return jsonify(info)

if __name__ == '__main__':
    import logging
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)
    app.run(host=SERVER_HOST, port=SERVER_PORT, threaded=True)