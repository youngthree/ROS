import os
import yaml
import asyncio
import threading
from typing import Dict, Any
from fastapi import FastAPI, Request, Response, status
from fastapi.responses import StreamingResponse, JSONResponse
from pydantic import BaseModel
from kubernetes import client, config, watch
from kubernetes.client.rest import ApiException

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Configuration loading
EDGEDEVICE_NAME = os.environ["EDGEDEVICE_NAME"]
EDGEDEVICE_NAMESPACE = os.environ["EDGEDEVICE_NAMESPACE"]
DEVICE_SHIFU_ADDRESS = None # Will be fetched from CRD

HTTP_SERVER_HOST = os.environ.get("HTTP_SERVER_HOST", "0.0.0.0")
HTTP_SERVER_PORT = int(os.environ.get("HTTP_SERVER_PORT", "8080"))

INSTRUCTION_CONFIG_PATH = "/etc/edgedevice/config/instructions"

PHASE_PENDING = "Pending"
PHASE_RUNNING = "Running"
PHASE_FAILED = "Failed"
PHASE_UNKNOWN = "Unknown"

# ROS2 setup (single node, reused across requests)
ros2_node = None
ros2_ready = threading.Event()

class ROS2CmdVelPublisher(Node):
    def __init__(self, topic, qos_profile=10):
        super().__init__('device_shifu_ros2_cmdvel')
        self.publisher_ = self.create_publisher(Twist, topic, qos_profile)
    def publish(self, twist: Twist):
        self.publisher_.publish(twist)

def ros2_thread_main(topic):
    global ros2_node
    rclpy.init()
    ros2_node = ROS2CmdVelPublisher(topic)
    ros2_ready.set()
    rclpy.spin(ros2_node)
    rclpy.shutdown()

def get_instruction_config() -> Dict[str, Any]:
    try:
        with open(INSTRUCTION_CONFIG_PATH, "r") as f:
            return yaml.safe_load(f)
    except Exception:
        return {}

def get_k8s_api():
    try:
        config.load_incluster_config()
    except Exception:
        config.load_kube_config()
    return client.CustomObjectsApi()

def get_edge_device(api) -> Dict[str, Any]:
    return api.get_namespaced_custom_object(
        "shifu.edgenesis.io", "v1alpha1", EDGEDEVICE_NAMESPACE, "edgedevices", EDGEDEVICE_NAME
    )

def update_edge_device_status(api, phase: str):
    status_body = {"status": {"edgeDevicePhase": phase}}
    try:
        api.patch_namespaced_custom_object_status(
            "shifu.edgenesis.io", "v1alpha1", EDGEDEVICE_NAMESPACE, "edgedevices", EDGEDEVICE_NAME, status_body
        )
    except ApiException:
        pass

def get_device_address_from_crd(api) -> str:
    try:
        dev = get_edge_device(api)
        return dev.get("spec", {}).get("address", None)
    except Exception:
        return None

def try_connect_ros2_and_update_status():
    api = get_k8s_api()
    update_edge_device_status(api, PHASE_PENDING)
    try:
        device_address = get_device_address_from_crd(api)
        if not device_address:
            update_edge_device_status(api, PHASE_UNKNOWN)
            return None
        instruction_cfg = get_instruction_config()
        # Assume topic is defined in instruction config for each API
        # Use a default if not found
        topic = "/cmd_vel"
        for api_name in instruction_cfg.values():
            if "protocolPropertyList" in api_name and "topic" in api_name["protocolPropertyList"]:
                topic = api_name["protocolPropertyList"]["topic"]
                break
        # Start ROS2 node in a thread if not already running
        if not ros2_ready.is_set():
            threading.Thread(target=ros2_thread_main, args=(topic,), daemon=True).start()
            # Wait for ROS2 node to be ready, timeout after 10s
            if not ros2_ready.wait(timeout=10):
                update_edge_device_status(api, PHASE_FAILED)
                return None
        update_edge_device_status(api, PHASE_RUNNING)
        return topic
    except Exception:
        update_edge_device_status(api, PHASE_FAILED)
        return None

# Ensure ROS2 node is running and update status on startup
ros2_topic = try_connect_ros2_and_update_status()

# FastAPI Server
app = FastAPI()

class MoveCommand(BaseModel):
    speed: float = 0.2  # m/s
    duration: float = 1 # seconds

def send_twist(linear_x, angular_z, speed, duration):
    global ros2_node
    if not ros2_ready.is_set():
        return False, "ROS2 node not ready"
    twist = Twist()
    twist.linear.x = linear_x * speed
    twist.angular.z = angular_z * speed
    ros2_node.publish(twist)
    # After duration, send zero command to stop
    def stop():
        import time
        time.sleep(duration)
        zero = Twist()
        ros2_node.publish(zero)
    threading.Thread(target=stop, daemon=True).start()
    return True, "Command sent"

@app.post("/move/forward")
async def move_forward(cmd: MoveCommand = None):
    cfg = get_instruction_config().get("move_forward", {}).get("protocolPropertyList", {})
    speed = float(cfg.get("speed", 0.2))
    duration = float(cfg.get("duration", 1))
    if cmd:
        speed = cmd.speed
        duration = cmd.duration
    ok, msg = send_twist(1.0, 0.0, speed, duration)
    if ok:
        return JSONResponse({"result": "moving forward", "speed": speed, "duration": duration})
    else:
        return JSONResponse({"error": msg}, status_code=status.HTTP_500_INTERNAL_SERVER_ERROR)

@app.post("/move/backward")
async def move_backward(cmd: MoveCommand = None):
    cfg = get_instruction_config().get("move_backward", {}).get("protocolPropertyList", {})
    speed = float(cfg.get("speed", 0.2))
    duration = float(cfg.get("duration", 1))
    if cmd:
        speed = cmd.speed
        duration = cmd.duration
    ok, msg = send_twist(-1.0, 0.0, speed, duration)
    if ok:
        return JSONResponse({"result": "moving backward", "speed": speed, "duration": duration})
    else:
        return JSONResponse({"error": msg}, status_code=status.HTTP_500_INTERNAL_SERVER_ERROR)

@app.post("/turn/left")
async def turn_left(cmd: MoveCommand = None):
    cfg = get_instruction_config().get("turn_left", {}).get("protocolPropertyList", {})
    speed = float(cfg.get("speed", 0.5))
    duration = float(cfg.get("duration", 1))
    if cmd:
        speed = cmd.speed
        duration = cmd.duration
    ok, msg = send_twist(0.0, 1.0, speed, duration)
    if ok:
        return JSONResponse({"result": "turning left", "speed": speed, "duration": duration})
    else:
        return JSONResponse({"error": msg}, status_code=status.HTTP_500_INTERNAL_SERVER_ERROR)

@app.post("/turn/right")
async def turn_right(cmd: MoveCommand = None):
    cfg = get_instruction_config().get("turn_right", {}).get("protocolPropertyList", {})
    speed = float(cfg.get("speed", 0.5))
    duration = float(cfg.get("duration", 1))
    if cmd:
        speed = cmd.speed
        duration = cmd.duration
    ok, msg = send_twist(0.0, -1.0, speed, duration)
    if ok:
        return JSONResponse({"result": "turning right", "speed": speed, "duration": duration})
    else:
        return JSONResponse({"error": msg}, status_code=status.HTTP_500_INTERNAL_SERVER_ERROR)

# Health endpoint
@app.get("/healthz")
async def health():
    return JSONResponse({"status": "ok", "ros2_ready": ros2_ready.is_set()})

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host=HTTP_SERVER_HOST, port=HTTP_SERVER_PORT)