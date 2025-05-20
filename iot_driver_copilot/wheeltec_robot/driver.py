import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from fastapi import FastAPI, Response
import uvicorn
import asyncio

# Configuration from environment variables
ROBOT_TOPIC = os.environ.get("ROBOT_CMD_VEL_TOPIC", "/cmd_vel")
HTTP_HOST = os.environ.get("HTTP_SERVER_HOST", "0.0.0.0")
HTTP_PORT = int(os.environ.get("HTTP_SERVER_PORT", "8000"))
ROS_DOMAIN_ID = os.environ.get("ROS_DOMAIN_ID", None)

# Set DDS domain ID if provided
if ROS_DOMAIN_ID is not None:
    os.environ["ROS_DOMAIN_ID"] = ROS_DOMAIN_ID

# Constants for movement
LINEAR_SPEED = float(os.environ.get("ROBOT_LINEAR_SPEED", "0.2"))
ANGULAR_SPEED = float(os.environ.get("ROBOT_ANGULAR_SPEED", "0.5"))

app = FastAPI()

class ROSPublisher(Node):
    def __init__(self, topic):
        super().__init__('wheeltec_http_driver')
        self.publisher_ = self.create_publisher(Twist, topic, 10)

    def send_cmd(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher_.publish(msg)

rclpy.init(args=None)
ros_node = ROSPublisher(ROBOT_TOPIC)

@app.get("/move/forward")
async def move_forward():
    ros_node.send_cmd(linear_x=LINEAR_SPEED, angular_z=0.0)
    return {"status": "ok", "action": "forward"}

@app.get("/move/backward")
async def move_backward():
    ros_node.send_cmd(linear_x=-LINEAR_SPEED, angular_z=0.0)
    return {"status": "ok", "action": "backward"}

@app.get("/turn/left")
async def turn_left():
    ros_node.send_cmd(linear_x=0.0, angular_z=ANGULAR_SPEED)
    return {"status": "ok", "action": "left"}

@app.get("/turn/right")
async def turn_right():
    ros_node.send_cmd(linear_x=0.0, angular_z=-ANGULAR_SPEED)
    return {"status": "ok", "action": "right"}

@app.get("/stop")
async def stop():
    ros_node.send_cmd(linear_x=0.0, angular_z=0.0)
    return {"status": "ok", "action": "stop"}

# Background ROS spin to handle DDS events
def ros_spin():
    while rclpy.ok():
        rclpy.spin_once(ros_node, timeout_sec=0.1)

def main():
    import threading
    t = threading.Thread(target=ros_spin, daemon=True)
    t.start()
    uvicorn.run(app, host=HTTP_HOST, port=HTTP_PORT)

if __name__ == '__main__':
    main()