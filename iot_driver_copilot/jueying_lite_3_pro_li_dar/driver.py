import os
import sys
import yaml
import asyncio
import logging
import socket
import struct
import threading
from fastapi import FastAPI, Request, Response, status
from fastapi.responses import JSONResponse
from kubernetes import client, config
from kubernetes.client.rest import ApiException
from pydantic import BaseModel
from typing import Optional, Dict, Any

# -------- CONFIGURATION & LOGGING --------

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)]
)
logger = logging.getLogger(__name__)

# Required environment variables
EDGEDEVICE_NAME = os.environ.get("EDGEDEVICE_NAME")
EDGEDEVICE_NAMESPACE = os.environ.get("EDGEDEVICE_NAMESPACE")
DEVICE_SERVER_HOST = os.environ.get("DEVICE_SERVER_HOST", "0.0.0.0")
DEVICE_SERVER_PORT = int(os.environ.get("DEVICE_SERVER_PORT", "8080"))
DEVICE_UDP_IP = os.environ.get("DEVICE_UDP_IP")
DEVICE_UDP_PORT = int(os.environ.get("DEVICE_UDP_PORT", "9000"))  # Default UDP port for robot
KUBERNETES_INCLUSTER = os.environ.get("KUBERNETES_SERVICE_HOST", None) is not None

CONFIG_INSTRUCTION_PATH = "/etc/edgedevice/config/instructions"

if not EDGEDEVICE_NAME or not EDGEDEVICE_NAMESPACE:
    logger.error("EDGEDEVICE_NAME and EDGEDEVICE_NAMESPACE environment variables are required")
    sys.exit(1)

if not DEVICE_UDP_IP or not DEVICE_UDP_PORT:
    logger.error("DEVICE_UDP_IP and DEVICE_UDP_PORT environment variables are required")
    sys.exit(1)

# -------- LOAD CONFIGMAP INSTRUCTION --------
def load_instruction_config() -> Dict[str, Any]:
    try:
        with open(CONFIG_INSTRUCTION_PATH, "r") as f:
            cfg = yaml.safe_load(f)
            return cfg if cfg else {}
    except Exception as ex:
        logger.warning(f"Failed to load instruction config: {ex}")
        return {}

instruction_config = load_instruction_config()

# -------- K8S CLIENT SETUP --------
def get_k8s_client():
    try:
        if KUBERNETES_INCLUSTER:
            config.load_incluster_config()
        else:
            config.load_kube_config()
        return client.CustomObjectsApi()
    except Exception as e:
        logger.error(f"Failed to load Kubernetes config: {e}")
        return None

k8s_client = get_k8s_client()

# ----------- DEVICE STATUS MANAGEMENT -----------

class DevicePhase:
    PENDING = "Pending"
    RUNNING = "Running"
    FAILED = "Failed"
    UNKNOWN = "Unknown"

def update_edge_device_phase(phase: str):
    if not k8s_client:
        logger.error("Kubernetes client not initialized, cannot update status")
        return
    try:
        body = {
            "status": {
                "edgeDevicePhase": phase
            }
        }
        k8s_client.patch_namespaced_custom_object_status(
            group="shifu.edgenesis.io",
            version="v1alpha1",
            namespace=EDGEDEVICE_NAMESPACE,
            plural="edgedevices",
            name=EDGEDEVICE_NAME,
            body=body
        )
        logger.info(f"EdgeDevice phase updated to: {phase}")
    except ApiException as ex:
        logger.error(f"Failed to update EdgeDevice status: {ex}")

def get_device_address_from_crd() -> Optional[str]:
    if not k8s_client:
        logger.error("Kubernetes client not initialized")
        return None
    try:
        obj = k8s_client.get_namespaced_custom_object(
            group="shifu.edgenesis.io",
            version="v1alpha1",
            namespace=EDGEDEVICE_NAMESPACE,
            plural="edgedevices",
            name=EDGEDEVICE_NAME
        )
        return obj.get("spec", {}).get("address", None)
    except Exception as ex:
        logger.warning(f"Could not get device address from CRD: {ex}")
        return None

# ----------- UDP COMMAND SENDER -----------

class UDPCommandSender:
    def __init__(self, ip: str, port: int, timeout: float = 2.0):
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(self.timeout)

    def send_command(self, cmd: str, payload: Optional[dict] = None) -> Dict[str, Any]:
        """
        Send a command via UDP, receive (or not) a response.
        Protocol: <cmd>|<payload-as-json>
        """
        try:
            msg = cmd
            if payload:
                import json
                msg += "|" + json.dumps(payload)
            self.sock.sendto(msg.encode('utf-8'), (self.ip, self.port))
            # Try to receive a response, if protocol supports.
            try:
                data, _ = self.sock.recvfrom(4096)
                return {"status": "success", "response": data.decode('utf-8')}
            except socket.timeout:
                return {"status": "success", "response": "No response (timeout)"}
        except Exception as ex:
            logger.error(f"UDP send command failed: {ex}")
            return {"status": "error", "error": str(ex)}

    def test_connectivity(self) -> bool:
        try:
            self.sock.sendto(b"PING", (self.ip, self.port))
            self.sock.settimeout(1.0)
            try:
                data, _ = self.sock.recvfrom(1024)
                return data.decode('utf-8').strip().upper() == "PONG"
            except socket.timeout:
                return False
        except Exception:
            return False

udp_sender = UDPCommandSender(DEVICE_UDP_IP, DEVICE_UDP_PORT)

# ----------- DEVICE STATUS BACKGROUND THREAD -----------

def device_status_monitor():
    prev_phase = None
    while True:
        try:
            # Test UDP connectivity: send PING, expect PONG
            if udp_sender.test_connectivity():
                phase = DevicePhase.RUNNING
            else:
                phase = DevicePhase.PENDING
        except Exception:
            phase = DevicePhase.FAILED
        if phase != prev_phase:
            update_edge_device_phase(phase)
            prev_phase = phase
        # Sleep for 10 seconds between checks
        threading.Event().wait(10)

status_thread = threading.Thread(target=device_status_monitor, daemon=True)
status_thread.start()

# ----------- FASTAPI SERVER -----------

app = FastAPI(
    title="Jueying Lite3 Pro LiDAR DeviceShifu",
    description="HTTP API for controlling Jueying Lite3 Pro LiDAR robot",
    version="1.0"
)

def get_protocol_settings(api_name: str) -> dict:
    return instruction_config.get(api_name, {}).get("protocolPropertyList", {})

class MovementRequest(BaseModel):
    speed: Optional[float] = None
    duration: Optional[float] = None
    angle: Optional[float] = None

def make_udp_api(api_name: str, cmd: str):
    async def handler(request: Request):
        try:
            body = await request.json()
        except Exception:
            body = {}
        protocol_settings = get_protocol_settings(api_name)
        # Merge protocol settings and body (body can override)
        payload = {**protocol_settings, **body}
        result = udp_sender.send_command(cmd, payload)
        if result["status"] == "success":
            return JSONResponse(
                status_code=status.HTTP_200_OK,
                content={"message": f"{cmd} command sent", "device_response": result["response"]}
            )
        else:
            return JSONResponse(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                content={"message": f"Failed to send {cmd} command", "error": result["error"]}
            )
    return handler

# API: /move/forward
app.add_api_route(
    "/move/forward",
    make_udp_api("move_forward", "MOVE_FORWARD"),
    methods=["POST"],
    response_class=JSONResponse,
    summary="Commands the LiDAR robot to move forward."
)

# API: /move/backward
app.add_api_route(
    "/move/backward",
    make_udp_api("move_backward", "MOVE_BACKWARD"),
    methods=["POST"],
    response_class=JSONResponse,
    summary="Commands the LiDAR robot to move backward."
)

# API: /turn/left
app.add_api_route(
    "/turn/left",
    make_udp_api("turn_left", "TURN_LEFT"),
    methods=["POST"],
    response_class=JSONResponse,
    summary="Commands the LiDAR robot to turn left."
)

# API: /turn/right
app.add_api_route(
    "/turn/right",
    make_udp_api("turn_right", "TURN_RIGHT"),
    methods=["POST"],
    response_class=JSONResponse,
    summary="Commands the LiDAR robot to turn right."
)

# ----------- HTTP ROOT -----------

@app.get("/")
def root():
    return {
        "device": "Jueying Lite3 Pro LiDAR",
        "manufacturer": "Jueying",
        "apis": ["/move/forward", "/move/backward", "/turn/left", "/turn/right"]
    }

# ----------- MAIN -----------

def main():
    import uvicorn
    uvicorn.run(app, host=DEVICE_SERVER_HOST, port=DEVICE_SERVER_PORT)

if __name__ == "__main__":
    main()