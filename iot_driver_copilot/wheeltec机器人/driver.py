import os
import json
import asyncio
from fastapi import FastAPI, Response
from fastapi.responses import StreamingResponse, JSONResponse
from starlette.requests import Request
import uvicorn

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Environment variable configuration
ROS_DOMAIN_ID = os.getenv("ROS_DOMAIN_ID", "0")
ROBOT_CMD_TOPIC = os.getenv("ROBOT_CMD_TOPIC", "/cmd_vel")
SERVER_HOST = os.getenv("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.getenv("SERVER_PORT", "8000"))

# Ensure ROS_DOMAIN_ID propagates before rclpy.init
os.environ["ROS_DOMAIN_ID"] = str(ROS_DOMAIN_ID)

# Command velocities for each action
CMD_MAP = {
    "move/forward":  {"linear": 0.2, "angular": 0.0},
    "move/backward": {"linear": -0.2, "angular": 0.0},
    "turn/left":     {"linear": 0.0, "angular": 0.6},
    "turn/right":    {"linear": 0.0, "angular": -0.6},
    "stop":          {"linear": 0.0, "angular": 0.0},
}

# Minimal ROS2 Publisher Node
class CmdVelPublisher(Node):
    def __init__(self, topic):
        super().__init__('http_server_cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, topic, 10)

    def send_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.publisher_.publish(msg)

# Singleton pattern for ROS2 node
class ROS2Context:
    _node = None

    @classmethod
    def get_node(cls):
        if cls._node is None:
            rclpy.init(args=None)
            cls._node = CmdVelPublisher(ROBOT_CMD_TOPIC)
        return cls._node

app = FastAPI()

@app.on_event("shutdown")
def shutdown_event():
    if ROS2Context._node is not None:
        ROS2Context._node.destroy_node()
        rclpy.shutdown()

def make_response(action: str):
    node = ROS2Context.get_node()
    cmd = CMD_MAP.get(action)
    if not cmd:
        return JSONResponse(status_code=404, content={"error": "Unknown command"})
    node.send_cmd(cmd["linear"], cmd["angular"])
    return JSONResponse(content={"status": "success", "action": action})

@app.get("/move/forward")
async def move_forward():
    return make_response("move/forward")

@app.get("/move/backward")
async def move_backward():
    return make_response("move/backward")

@app.get("/turn/left")
async def turn_left():
    return make_response("turn/left")

@app.get("/turn/right")
async def turn_right():
    return make_response("turn/right")

@app.get("/stop")
async def stop():
    return make_response("stop")

if __name__ == "__main__":
    uvicorn.run(app, host=SERVER_HOST, port=SERVER_PORT)