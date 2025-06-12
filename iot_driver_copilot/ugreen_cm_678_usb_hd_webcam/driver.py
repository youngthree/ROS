import os
import threading
import queue
import time
import json
from flask import Flask, jsonify

import paho.mqtt.client as mqtt

MQTT_BROKER_ADDRESS = os.environ["MQTT_BROKER_ADDRESS"]
MQTT_CLIENT_ID = os.environ.get("MQTT_CLIENT_ID", "ugreen-cm678-shifu")
MQTT_USERNAME = os.environ.get("MQTT_USERNAME")
MQTT_PASSWORD = os.environ.get("MQTT_PASSWORD")
MQTT_KEEPALIVE = int(os.environ.get("MQTT_KEEPALIVE", 60))

TOPIC_PROBE = "/probe"
TOPIC_VIDEO = "/video"
TOPIC_CAPTURE = "/capture"
QOS_PROBE = 1
QOS_VIDEO = 1
QOS_CAPTURE = 1

# Queues to store the latest data for each topic
data_queues = {
    TOPIC_PROBE: queue.Queue(maxsize=1),
    TOPIC_VIDEO: queue.Queue(maxsize=1),
    TOPIC_CAPTURE: queue.Queue(maxsize=1)
}

def _update_queue(topic, payload):
    q = data_queues[topic]
    try:
        if not q.empty():
            q.get_nowait()
    except Exception:
        pass
    q.put(payload)

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        client.subscribe([(TOPIC_PROBE, QOS_PROBE), (TOPIC_VIDEO, QOS_VIDEO), (TOPIC_CAPTURE, QOS_CAPTURE)])
    else:
        print(f"MQTT connect failed with code {rc}")

def on_message(client, userdata, msg):
    topic = msg.topic
    payload = msg.payload.decode('utf-8')
    if topic in data_queues:
        _update_queue(topic, payload)

def mqtt_thread():
    client = mqtt.Client(client_id=MQTT_CLIENT_ID)
    if MQTT_USERNAME:
        client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.on_connect = on_connect
    client.on_message = on_message

    broker, port = MQTT_BROKER_ADDRESS.split(":")
    port = int(port)
    client.connect(broker, port, keepalive=MQTT_KEEPALIVE)
    client.loop_forever()

# Flask App
app = Flask(__name__)

def get_latest_data(topic):
    q = data_queues[topic]
    try:
        payload = q.get(timeout=5)
        q.put(payload)
        return jsonify(json.loads(payload))
    except queue.Empty:
        return jsonify({"error": "No data available"}), 504
    except Exception as e:
        return jsonify({"error": "Invalid payload", "details": str(e)}), 500

@app.route("/probe", methods=["GET"])
def get_probe():
    return get_latest_data(TOPIC_PROBE)

@app.route("/video", methods=["GET"])
def get_video():
    return get_latest_data(TOPIC_VIDEO)

@app.route("/capture", methods=["GET"])
def get_capture():
    return get_latest_data(TOPIC_CAPTURE)

if __name__ == "__main__":
    t = threading.Thread(target=mqtt_thread, daemon=True)
    t.start()
    app.run(host="0.0.0.0", port=int(os.environ.get("SHIFU_HTTP_PORT", 8080)))