import os
import json
import threading
import time
import queue
import paho.mqtt.client as mqtt

class JueyingLite3ProMqttDriver:
    def __init__(self):
        self.mqtt_broker_address = os.environ.get("MQTT_BROKER_ADDRESS")
        if not self.mqtt_broker_address:
            raise EnvironmentError("MQTT_BROKER_ADDRESS environment variable must be set")
        self.mqtt_client_id = os.environ.get("MQTT_CLIENT_ID", "jueying-lite3pro-shifu")
        self.mqtt_username = os.environ.get("MQTT_USERNAME")
        self.mqtt_password = os.environ.get("MQTT_PASSWORD")
        self.odom_queue = queue.Queue()
        self._setup_mqtt_client()
        self._subscribe_odom()

    def _setup_mqtt_client(self):
        self.client = mqtt.Client(client_id=self.mqtt_client_id, clean_session=True)
        if self.mqtt_username and self.mqtt_password:
            self.client.username_pw_set(self.mqtt_username, self.mqtt_password)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.connect(self._parse_host(), self._parse_port(), keepalive=60)
        self.client.loop_start()

    def _parse_host(self):
        # Supports host:port or just host
        if ":" in self.mqtt_broker_address:
            return self.mqtt_broker_address.split(":")[0]
        return self.mqtt_broker_address

    def _parse_port(self):
        if ":" in self.mqtt_broker_address:
            return int(self.mqtt_broker_address.split(":")[1])
        return 1883  # default MQTT port

    def _on_connect(self, client, userdata, flags, rc):
        pass  # Connection success/failure can be handled/logged here if needed

    def _on_message(self, client, userdata, msg):
        if msg.topic == "/odom":
            try:
                payload = msg.payload.decode('utf-8')
                json_data = json.loads(payload)
                self.odom_queue.put(json_data)
            except Exception:
                pass

    def _subscribe_odom(self):
        # Subscribe to /odom with QoS 1
        self.client.subscribe("/odom", qos=1)

    # Device API methods

    def move_forward(self, speed):
        # speed: float or int, required
        payload = {"speed": speed}
        return self._publish("/move/forward", payload, qos=1)

    def move_backward(self, speed):
        payload = {"speed": speed}
        return self._publish("/move/backward", payload, qos=1)

    def turn_left(self, angle):
        payload = {"angle": angle}
        return self._publish("/turn/left", payload, qos=1)

    def turn_right(self, angle):
        payload = {"angle": angle}
        return self._publish("/turn/right", payload, qos=1)

    def stop(self):
        payload = {}
        return self._publish("/stop", payload, qos=1)

    def cmd_vel(self, linear_x, angular_z):
        payload = {
            "linear": {"x": linear_x},
            "angular": {"z": angular_z}
        }
        return self._publish("/cmd_vel", payload, qos=1)

    def get_odom(self, timeout=2.0):
        """
        Returns the latest odometry message received from the /odom topic.
        Waits up to `timeout` seconds for a message if none available.
        Returns: dict or None if no message in timeout.
        """
        try:
            return self.odom_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    # Internal publish helper
    def _publish(self, topic, payload, qos=1):
        try:
            json_payload = json.dumps(payload)
            result, mid = self.client.publish(topic, json_payload, qos=qos)
            # Wait for publish completion (optional: could provide feedback)
            if result == mqtt.MQTT_ERR_SUCCESS:
                return True
            else:
                return False
        except Exception:
            return False

    def close(self):
        self.client.loop_stop()
        self.client.disconnect()

# Example usage (would be replaced by DeviceShifu API handlers in real deployment)
if __name__ == "__main__":
    import sys
    import signal

    driver = JueyingLite3ProMqttDriver()

    def shutdown(signum, frame):
        driver.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    # Example: Move forward at speed 1.2
    driver.move_forward(1.2)
    # Example: Turn left by 30 degrees
    driver.turn_left(30)
    # Example: Stop
    driver.stop()
    # Example: Set velocity
    driver.cmd_vel(0.5, 0.2)
    # Example: Get odometry
    odom = driver.get_odom(timeout=5.0)
    print("Odometry:", odom)
    driver.close()