import os
import json
import threading
import time
from typing import Callable, Optional, Any, Dict

import paho.mqtt.client as mqtt

class WheeltecROS2MQTTDriver:
    def __init__(self):
        self.mqtt_broker = os.environ.get('MQTT_BROKER_ADDRESS')
        if not self.mqtt_broker:
            raise ValueError("MQTT_BROKER_ADDRESS environment variable is required.")
        self.mqtt_client_id = os.environ.get('MQTT_CLIENT_ID', 'wshifu_' + str(int(time.time())))
        self.mqtt_username = os.environ.get('MQTT_USERNAME')
        self.mqtt_password = os.environ.get('MQTT_PASSWORD')
        self.mqtt_keepalive = int(os.environ.get('MQTT_KEEPALIVE', '60'))

        self.client = mqtt.Client(client_id=self.mqtt_client_id, clean_session=True)
        if self.mqtt_username and self.mqtt_password:
            self.client.username_pw_set(self.mqtt_username, self.mqtt_password)

        self._odom_callback: Optional[Callable[[Dict[str, Any]], None]] = None
        self._odom_thread = None
        self._odom_stop_event = threading.Event()
        self._connected_event = threading.Event()

        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message

        self.client.connect(self._get_host(), self._get_port(), self.mqtt_keepalive)
        self.client.loop_start()
        self._connected_event.wait(timeout=10)

    def _get_host(self):
        parts = self.mqtt_broker.split(':')
        return parts[0]

    def _get_port(self):
        parts = self.mqtt_broker.split(':')
        return int(parts[1]) if len(parts) > 1 else 1883

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self._connected_event.set()
        else:
            raise ConnectionError(f"Failed to connect to MQTT broker: {rc}")

    def _on_message(self, client, userdata, msg):
        if msg.topic == "/odom" and self._odom_callback:
            try:
                payload = json.loads(msg.payload.decode('utf-8'))
                self._odom_callback(payload)
            except Exception:
                pass

    def publish_cmd_vel(self, linear_x: float, linear_y: float = 0.0, linear_z: float = 0.0,
                        angular_x: float = 0.0, angular_y: float = 0.0, angular_z: float = 0.0, qos: int = 1):
        payload = {
            "linear": {"x": linear_x, "y": linear_y, "z": linear_z},
            "angular": {"x": angular_x, "y": angular_y, "z": angular_z}
        }
        self.client.publish("/cmd_vel", json.dumps(payload), qos=qos)

    def move_forward(self, speed: float, duration: Optional[float] = None, qos: int = 1):
        payload = {"action": "forward", "speed": speed}
        if duration is not None:
            payload["duration"] = duration
        self.client.publish("/move/forward", json.dumps(payload), qos=qos)

    def move_backward(self, speed: float, duration: Optional[float] = None, qos: int = 1):
        payload = {"action": "backward", "speed": speed}
        if duration is not None:
            payload["duration"] = duration
        self.client.publish("/move/backward", json.dumps(payload), qos=qos)

    def turn_left(self, angular_velocity: float, angle: Optional[float] = None, qos: int = 1):
        payload = {"action": "left", "angular_velocity": angular_velocity}
        if angle is not None:
            payload["angle"] = angle
        self.client.publish("/turn/left", json.dumps(payload), qos=qos)

    def turn_right(self, angular_velocity: float, angle: Optional[float] = None, qos: int = 1):
        payload = {"action": "right", "angular_velocity": angular_velocity}
        if angle is not None:
            payload["angle"] = angle
        self.client.publish("/turn/right", json.dumps(payload), qos=qos)

    def stop(self, qos: int = 1):
        payload = {"action": "stop"}
        self.client.publish("/stop", json.dumps(payload), qos=qos)

    def subscribe_odom(self, callback: Callable[[Dict[str, Any]], None], qos: int = 1):
        """
        Subscribe to /odom topic. The callback will be called with the decoded JSON message.
        """
        self._odom_callback = callback
        self.client.subscribe("/odom", qos=qos)

    def unsubscribe_odom(self):
        self.client.unsubscribe("/odom")
        self._odom_callback = None

    def disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()


# Example usage (to be removed/commented out in deployment):
# driver = WheeltecROS2MQTTDriver()
# driver.move_forward(speed=0.3, duration=2)
# driver.turn_left(angular_velocity=0.5, angle=1.57)
# driver.stop()
# def odom_callback(msg):
#     print("Odom:", msg)
# driver.subscribe_odom(odom_callback)
# time.sleep(5)
# driver.unsubscribe_odom()
# driver.disconnect()