import os
import threading
import time
import queue
import json
from flask import Flask, jsonify, make_response

import paho.mqtt.client as mqtt

# Environment variable for MQTT broker
MQTT_BROKER_ADDRESS = os.environ.get("MQTT_BROKER_ADDRESS")
if not MQTT_BROKER_ADDRESS:
    raise EnvironmentError("MQTT_BROKER_ADDRESS environment variable is required.")

# Get MQTT auth (optional)
MQTT_USERNAME = os.environ.get("MQTT_USERNAME")
MQTT_PASSWORD = os.environ.get("MQTT_PASSWORD")
MQTT_CLIENT_ID = os.environ.get("MQTT_CLIENT_ID", "ugreen-cm678-shifu")

# MQTT Topics
VIDEO_TOPIC = "device/webcam/video"
CAPTURE_TOPIC = "device/webcam/capture"
PROBE_TOPIC = "device/webcam/probe"

# QoS
VIDEO_QOS = 1
CAPTURE_QOS = 1
PROBE_QOS = 1

# Message buffers (max 10 messages kept, drop oldest)
class MessageBuffer:
    def __init__(self, maxlen=10):
        self.q = queue.Queue(maxlen)
        self.lock = threading.Lock()
        self.last = None  # Always keep last message for GET

    def put(self, value):
        with self.lock:
            if self.q.full():
                try:
                    self.q.get_nowait()
                except queue.Empty:
                    pass
            self.q.put_nowait(value)
            self.last = value

    def get_last(self):
        with self.lock:
            return self.last

video_buffer = MessageBuffer()
capture_buffer = MessageBuffer()
probe_buffer = MessageBuffer()

# MQTT callbacks
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        client.subscribe([(VIDEO_TOPIC, VIDEO_QOS), (CAPTURE_TOPIC, CAPTURE_QOS), (PROBE_TOPIC, PROBE_QOS)])
    else:
        print("Failed to connect to MQTT broker, return code:", rc)

def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode("utf-8")
        # Try to parse JSON (if not, just keep as string)
        try:
            data = json.loads(payload)
        except Exception:
            data = payload
        if msg.topic == VIDEO_TOPIC:
            video_buffer.put(data)
        elif msg.topic == CAPTURE_TOPIC:
            capture_buffer.put(data)
        elif msg.topic == PROBE_TOPIC:
            probe_buffer.put(data)
    except Exception as e:
        print("Error processing MQTT message:", e)

# MQTT Client Setup
mqtt_client = mqtt.Client(client_id=MQTT_CLIENT_ID)
if MQTT_USERNAME and MQTT_PASSWORD:
    mqtt_client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message

def mqtt_thread_fn():
    host_port = MQTT_BROKER_ADDRESS.split(":")
    mqtt_host = host_port[0]
    mqtt_port = int(host_port[1]) if len(host_port) > 1 else 1883
    while True:
        try:
            mqtt_client.connect(mqtt_host, mqtt_port, 60)
            mqtt_client.loop_forever()
        except Exception as e:
            print("MQTT connection error:", e)
            time.sleep(3)

mqtt_thread = threading.Thread(target=mqtt_thread_fn, daemon=True)
mqtt_thread.start()

# Flask REST API
app = Flask(__name__)

@app.route("/video", methods=["GET"])
def get_video():
    data = video_buffer.get_last()
    if data is not None:
        return make_response(jsonify(data), 200)
    return make_response(jsonify({"error": "No video stream data available"}), 404)

@app.route("/capture", methods=["GET"])
def get_capture():
    data = capture_buffer.get_last()
    if data is not None:
        return make_response(jsonify(data), 200)
    return make_response(jsonify({"error": "No captured image data available"}), 404)

@app.route("/probe", methods=["GET"])
def get_probe():
    data = probe_buffer.get_last()
    if data is not None:
        return make_response(jsonify(data), 200)
    return make_response(jsonify({"error": "No device probe data available"}), 404)

if __name__ == "__main__":
    port = int(os.environ.get("DRIVER_REST_PORT", "8080"))
    app.run(host="0.0.0.0", port=port)