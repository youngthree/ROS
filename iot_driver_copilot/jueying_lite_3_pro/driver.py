import os
import asyncio
import json
from typing import Optional

from fastapi import FastAPI, Request, Response, status
from fastapi.responses import StreamingResponse, JSONResponse
from pydantic import BaseModel

import socket
import struct

# ---- ENV CONFIG ----

ROBOT_IP = os.environ.get("ROBOT_IP")
ROBOT_UDP_PORT = int(os.environ.get("ROBOT_UDP_PORT", "9000"))
HTTP_HOST = os.environ.get("HTTP_HOST", "0.0.0.0")
HTTP_PORT = int(os.environ.get("HTTP_PORT", "8080"))

if not ROBOT_IP:
    raise RuntimeError("ROBOT_IP environment variable must be set.")

# ---- UDP COMMUNICATION HELPERS ----

class UDPClient:
    def __init__(self, ip: str, port: int, timeout: float = 2.0):
        self.addr = (ip, port)
        self.timeout = timeout

    async def send_and_receive(self, msg: bytes, expected_len: Optional[int] = None) -> bytes:
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(None, self._sync_send_and_receive, msg, expected_len)

    def _sync_send_and_receive(self, msg: bytes, expected_len: Optional[int]):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.settimeout(self.timeout)
            s.sendto(msg, self.addr)
            try:
                data, _ = s.recvfrom(expected_len or 65536)
                return data
            except socket.timeout:
                raise TimeoutError("No response from robot via UDP.")

udp_client = UDPClient(ROBOT_IP, ROBOT_UDP_PORT)

# ---- ROS/UDP MESSAGE ENCODING/DECODING ----

# For demonstration, we use simple JSON over UDP for both /state (request/response) and /move (command/ack)
# In a real implementation, encode/decode according to the robot's binary or ROS protocol.

# ---- FASTAPI MODELS ----

class MoveCommand(BaseModel):
    linear_x: float
    linear_y: Optional[float] = 0.0
    linear_z: Optional[float] = 0.0
    angular_x: Optional[float] = 0.0
    angular_y: Optional[float] = 0.0
    angular_z: float

# ---- FASTAPI APP ----

app = FastAPI()

@app.get("/state")
async def get_state():
    # Send a request for current state (example: '{"cmd":"get_state"}')
    request_msg = json.dumps({"cmd": "get_state"}).encode("utf-8")
    try:
        raw = await udp_client.send_and_receive(request_msg)
        # Assume response is JSON-encoded
        try:
            data = json.loads(raw.decode("utf-8"))
            return JSONResponse(content=data)
        except Exception:
            # If not JSON, just return as a binary blob
            return Response(content=raw, media_type="application/octet-stream")
    except TimeoutError:
        return JSONResponse(status_code=status.HTTP_504_GATEWAY_TIMEOUT, content={"error": "No response from robot"})

@app.post("/move")
async def post_move(cmd: MoveCommand):
    # Send move command to robot (example: '{"cmd":"move", "vel": {...}}')
    move_payload = {
        "cmd": "move",
        "vel": {
            "linear": {
                "x": cmd.linear_x,
                "y": cmd.linear_y,
                "z": cmd.linear_z
            },
            "angular": {
                "x": cmd.angular_x,
                "y": cmd.angular_y,
                "z": cmd.angular_z
            }
        }
    }
    request_msg = json.dumps(move_payload).encode("utf-8")
    try:
        raw = await udp_client.send_and_receive(request_msg)
        try:
            data = json.loads(raw.decode("utf-8"))
            return JSONResponse(content=data)
        except Exception:
            return Response(content=raw, media_type="application/octet-stream")
    except TimeoutError:
        return JSONResponse(status_code=status.HTTP_504_GATEWAY_TIMEOUT, content={"error": "No response from robot"})

# ---- MAIN ENTRYPOINT ----

def main():
    import uvicorn
    uvicorn.run(app, host=HTTP_HOST, port=HTTP_PORT)

if __name__ == "__main__":
    main()
