import os
from flask import Flask, Response, jsonify, send_file
import threading
import queue
import paho.mqtt.client as mqtt
import io
import time
import json
from werkzeug.exceptions import ServiceUnavailable

app = Flask(__name__)

# Environment variable configuration
MQTT_BROKER_ADDRESS = os.environ.get("MQTT_BROKER_ADDRESS")
if not MQTT_BROKER_ADDRESS:
    raise EnvironmentError("MQTT_BROKER_ADDRESS environment variable is required")

MQTT_CLIENT_ID = os.environ.get("MQTT_CLIENT_ID", "webcam_shifu")
MQTT_USERNAME = os.environ.get("MQTT_USERNAME")
MQTT_PASSWORD = os.environ.get("MQTT_PASSWORD")
MQTT_KEEPALIVE = int(os.environ.get("MQTT_KEEPALIVE", "60"))

TOPIC_VIDEO = "device/webcam/video"
TOPIC_CAPTURE = "device/webcam/capture"
TOPIC_PROBE = "device/webcam/probe"

# Internal queues for MQTT messages
video_queue = queue.Queue(maxsize=10)
capture_queue = queue.Queue(maxsize=1)
probe_queue = queue.Queue(maxsize=1)

# MQTT callbacks
def on_connect(client, userdata, flags, rc):
    # Subscribe to all topics on connect
    client.subscribe([(TOPIC_VIDEO, 1), (TOPIC_CAPTURE, 1), (TOPIC_PROBE, 1)])

def on_message(client, userdata, msg):
    if msg.topic == TOPIC_VIDEO:
        # For video streaming, keep only last N frames to avoid memory bloat
        try:
            video_queue.put_nowait(msg.payload)
        except queue.Full:
            try:
                video_queue.get_nowait()
            except queue.Empty:
                pass
            video_queue.put_nowait(msg.payload)
    elif msg.topic == TOPIC_CAPTURE:
        # For capture, keep only the latest image
        while not capture_queue.empty():
            capture_queue.get_nowait()
        capture_queue.put_nowait(msg.payload)
    elif msg.topic == TOPIC_PROBE:
        # For probe/status, keep only the latest status
        while not probe_queue.empty():
            probe_queue.get_nowait()
        probe_queue.put_nowait(msg.payload)

# MQTT client setup
def mqtt_thread():
    client = mqtt.Client(client_id=MQTT_CLIENT_ID)
    if MQTT_USERNAME:
        client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.on_connect = on_connect
    client.on_message = on_message

    broker_host, broker_port = MQTT_BROKER_ADDRESS.rsplit(":", 1)
    client.connect(broker_host, int(broker_port), keepalive=MQTT_KEEPALIVE)
    client.loop_forever()

mqtt_bg_thread = threading.Thread(target=mqtt_thread, daemon=True)
mqtt_bg_thread.start()

# Flask endpoints

@app.route("/video", methods=["GET"])
def get_video():
    def generate():
        boundary = "frame"
        # Start MJPEG multipart stream
        while True:
            try:
                frame = video_queue.get(timeout=10)
                yield (b'--' + boundary.encode() + b'\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' +
                       frame + b'\r\n')
            except queue.Empty:
                break
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/capture", methods=["GET"])
def get_capture():
    try:
        image_data = capture_queue.get(timeout=5)
        # Try to guess image format from first bytes
        if image_data[:8] == b"\x89PNG\r\n\x1a\n":
            mimetype = "image/png"
            ext = "png"
        elif image_data[:2] == b"\xff\xd8":
            mimetype = "image/jpeg"
            ext = "jpg"
        else:
            mimetype = "application/octet-stream"
            ext = "bin"
        return Response(image_data, mimetype=mimetype,
                        headers={"Content-Disposition": f"inline; filename=capture.{ext}"})
    except queue.Empty:
        raise ServiceUnavailable("No capture image available from webcam.")

@app.route("/probe", methods=["GET"])
def get_probe():
    try:
        status_data = probe_queue.get(timeout=5)
        # Should be JSON
        status_json = json.loads(status_data.decode("utf-8"))
        return jsonify(status_json)
    except queue.Empty:
        raise ServiceUnavailable("No probe/status data available from webcam.")
    except Exception:
        raise ServiceUnavailable("Invalid probe/status data received.")

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=int(os.environ.get("SHIFU_DRIVER_PORT", "8080")))