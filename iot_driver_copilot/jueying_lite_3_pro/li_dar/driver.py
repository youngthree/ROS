import os
import asyncio
import json
from fastapi import FastAPI, Response, status
from fastapi.responses import StreamingResponse, JSONResponse
from pydantic import BaseModel
import uvicorn
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# ====== ENVIRONMENT VARIABLE CONFIG ======
ROBOT_ROS_MASTER_URI = os.getenv('ROBOT_ROS_MASTER_URI', 'localhost')
ROBOT_ROS_NAMESPACE = os.getenv('ROBOT_ROS_NAMESPACE', '')
SERVER_HOST = os.getenv('SERVER_HOST', '0.0.0.0')
SERVER_PORT = int(os.getenv('SERVER_PORT', '8080'))
ODOM_TOPIC = os.getenv('ODOM_TOPIC', '/odom')
CMD_VEL_TOPIC = os.getenv('CMD_VEL_TOPIC', '/cmd_vel')
ROS_DOMAIN_ID = os.getenv('ROS_DOMAIN_ID', None)
if ROS_DOMAIN_ID is not None:
    os.environ['ROS_DOMAIN_ID'] = ROS_DOMAIN_ID

# ====== ROS2 NODE SETUP ======
class RobotCommandNode(Node):
    def __init__(self, odom_topic, cmd_vel_topic):
        super().__init__('web_cmd_node')
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.last_odom = None
        self.odom_event = asyncio.Event()
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

    def odom_callback(self, msg):
        self.last_odom = msg
        self.odom_event.set()

    def publish_cmd(self, linear_x=0.0, angular_z=0.0):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)

# ====== FASTAPI SETUP ======
app = FastAPI()
rclpy.init(args=None)
ros_node = RobotCommandNode(ODOM_TOPIC, CMD_VEL_TOPIC)

# ====== COMMAND MODELS ======
class CommandResponse(BaseModel):
    status: str

@app.post("/move/forward", response_model=CommandResponse)
async def move_forward():
    ros_node.publish_cmd(linear_x=0.3, angular_z=0.0)
    return {"status": "moving forward"}

@app.post("/move/backward", response_model=CommandResponse)
async def move_backward():
    ros_node.publish_cmd(linear_x=-0.3, angular_z=0.0)
    return {"status": "moving backward"}

@app.post("/turn/left", response_model=CommandResponse)
async def turn_left():
    ros_node.publish_cmd(linear_x=0.0, angular_z=0.5)
    return {"status": "turning left"}

@app.post("/turn/right", response_model=CommandResponse)
async def turn_right():
    ros_node.publish_cmd(linear_x=0.0, angular_z=-0.5)
    return {"status": "turning right"}

@app.post("/stop", response_model=CommandResponse)
async def stop():
    ros_node.publish_cmd(linear_x=0.0, angular_z=0.0)
    return {"status": "stopped"}

def odom_msg_to_dict(msg: Odometry):
    return {
        "header": {
            "stamp": {
                "sec": msg.header.stamp.sec,
                "nanosec": msg.header.stamp.nanosec
            },
            "frame_id": msg.header.frame_id
        },
        "child_frame_id": msg.child_frame_id,
        "pose": {
            "pose": {
                "position": {
                    "x": msg.pose.pose.position.x,
                    "y": msg.pose.pose.position.y,
                    "z": msg.pose.pose.position.z
                },
                "orientation": {
                    "x": msg.pose.pose.orientation.x,
                    "y": msg.pose.pose.orientation.y,
                    "z": msg.pose.pose.orientation.z,
                    "w": msg.pose.pose.orientation.w
                }
            },
            "covariance": list(msg.pose.covariance)
        },
        "twist": {
            "twist": {
                "linear": {
                    "x": msg.twist.twist.linear.x,
                    "y": msg.twist.twist.linear.y,
                    "z": msg.twist.twist.linear.z
                },
                "angular": {
                    "x": msg.twist.twist.angular.x,
                    "y": msg.twist.twist.angular.y,
                    "z": msg.twist.twist.angular.z
                }
            },
            "covariance": list(msg.twist.covariance)
        }
    }

@app.get("/odom")
async def get_odom():
    # Wait up to 2s for new odometry data
    try:
        await asyncio.wait_for(ros_node.odom_event.wait(), timeout=2.0)
        ros_node.odom_event.clear()
    except asyncio.TimeoutError:
        pass
    msg = ros_node.last_odom
    if msg is None:
        return JSONResponse({"status": "no odom available"}, status_code=status.HTTP_503_SERVICE_UNAVAILABLE)
    result = odom_msg_to_dict(msg)
    return JSONResponse(result)

# ====== RUNNER ======
if __name__ == "__main__":
    uvicorn.run("main:app", host=SERVER_HOST, port=SERVER_PORT, reload=False)