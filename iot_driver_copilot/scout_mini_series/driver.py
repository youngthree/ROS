import os
import io
import asyncio
import json
from typing import Optional
from fastapi import FastAPI, Request, Response, BackgroundTasks, HTTPException
from fastapi.responses import StreamingResponse, JSONResponse
from pydantic import BaseModel
import uvicorn

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, BatteryState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist

import cv2
import numpy as np

app = FastAPI()

# Environment Variables
ROS_DOMAIN_ID = int(os.environ.get("ROS_DOMAIN_ID", "0"))
ROS_NAMESPACE = os.environ.get("ROS_NAMESPACE", "")
ROBOT_STATUS_TIMEOUT = float(os.environ.get("ROBOT_STATUS_TIMEOUT", "1.0"))
CAMERA_TOPIC = os.environ.get("CAMERA_TOPIC", "/camera/color/image_raw")
LIDAR_TOPIC = os.environ.get("LIDAR_TOPIC", "/scan")
ODOM_TOPIC = os.environ.get("ODOM_TOPIC", "/odom")
BATTERY_TOPIC = os.environ.get("BATTERY_TOPIC", "/battery_state")
STATUS_TOPIC = os.environ.get("STATUS_TOPIC", "/robot/status")
NAVGOAL_TOPIC = os.environ.get("NAVGOAL_TOPIC", "/move_base_simple/goal")
MOVE_TOPIC = os.environ.get("MOVE_TOPIC", "/cmd_vel")
MAPSAVE_SERVICE = os.environ.get("MAPSAVE_SERVICE", "/map_saver/save_map")
HTTP_HOST = os.environ.get("HTTP_HOST", "0.0.0.0")
HTTP_PORT = int(os.environ.get("HTTP_PORT", "8080"))

# --- ROS2 Node Setup ---
class RobotBridge(Node):
    def __init__(self):
        super().__init__('scout_mini_driver')
        self.status_data = {}
        self.lidar_data = None
        self.odom_data = None
        self.battery_data = None
        self.camera_frame = None
        self.status_event = asyncio.Event()

        self.create_subscription(LaserScan, LIDAR_TOPIC, self.lidar_callback, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self.odom_callback, 10)
        self.create_subscription(BatteryState, BATTERY_TOPIC, self.battery_callback, 10)
        self.create_subscription(Image, CAMERA_TOPIC, self.camera_callback, 10)
        self.create_subscription(String, STATUS_TOPIC, self.status_callback, 10)

        self.move_pub = self.create_publisher(Twist, MOVE_TOPIC, 10)
        self.navgoal_pub = self.create_publisher(PoseStamped, NAVGOAL_TOPIC, 10)

    def lidar_callback(self, msg):
        self.lidar_data = {
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'range_min': msg.range_min,
            'range_max': msg.range_max,
            'ranges': list(msg.ranges),
            'intensities': list(msg.intensities)
        }
        self.status_event.set()

    def odom_callback(self, msg):
        self.odom_data = {
            'pose': {
                'position': {
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'z': msg.pose.pose.position.z
                },
                'orientation': {
                    'x': msg.pose.pose.orientation.x,
                    'y': msg.pose.pose.orientation.y,
                    'z': msg.pose.pose.orientation.z,
                    'w': msg.pose.pose.orientation.w
                }
            },
            'twist': {
                'linear': {
                    'x': msg.twist.twist.linear.x,
                    'y': msg.twist.twist.linear.y,
                    'z': msg.twist.twist.linear.z
                },
                'angular': {
                    'x': msg.twist.twist.angular.x,
                    'y': msg.twist.twist.angular.y,
                    'z': msg.twist.twist.angular.z
                }
            }
        }
        self.status_event.set()

    def battery_callback(self, msg):
        self.battery_data = {
            'voltage': msg.voltage,
            'current': msg.current,
            'charge': msg.charge,
            'capacity': msg.capacity,
            'percentage': msg.percentage
        }
        self.status_event.set()

    def status_callback(self, msg):
        try:
            self.status_data = json.loads(msg.data)
        except Exception:
            self.status_data = {"raw": msg.data}
        self.status_event.set()

    def camera_callback(self, msg):
        if msg.encoding not in ["rgb8", "bgr8"]:
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        _, jpeg = cv2.imencode(".jpg", img)
        self.camera_frame = jpeg.tobytes()

    def publish_move(self, linear: float, angular: float):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.move_pub.publish(twist)

    def publish_navgoal(self, x: float, y: float, yaw: float = 0.0):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        # yaw to quaternion
        qx, qy, qz, qw = 0, 0, np.sin(yaw/2), np.cos(yaw/2)
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw
        self.navgoal_pub.publish(goal)

robot_bridge: Optional[RobotBridge] = None

def ros2_spin():
    rclpy.spin(robot_bridge)

@app.on_event("startup")
async def startup_event():
    global robot_bridge
    rclpy.init(args=None)
    robot_bridge = RobotBridge()
    loop = asyncio.get_event_loop()
    loop.run_in_executor(None, ros2_spin)

# --- API Models ---
class MoveCommand(BaseModel):
    linear: float
    angular: float

class NavGoalCommand(BaseModel):
    x: float
    y: float
    yaw: Optional[float] = 0.0

# --- API Endpoints ---

@app.post("/move")
async def move(cmd: MoveCommand):
    if robot_bridge is None:
        raise HTTPException(status_code=500, detail="ROS bridge not initialized")
    robot_bridge.publish_move(cmd.linear, cmd.angular)
    return {"status": "moving", "linear": cmd.linear, "angular": cmd.angular}

@app.post("/navgoal")
async def navgoal(cmd: NavGoalCommand):
    if robot_bridge is None:
        raise HTTPException(status_code=500, detail="ROS bridge not initialized")
    robot_bridge.publish_navgoal(cmd.x, cmd.y, cmd.yaw)
    return {"status": "navigating", "goal": {"x": cmd.x, "y": cmd.y, "yaw": cmd.yaw}}

@app.post("/mapsave")
async def mapsave():
    if robot_bridge is None:
        raise HTTPException(status_code=500, detail="ROS bridge not initialized")
    # Call /map_saver/save_map service via ROS2
    from std_srvs.srv import Empty
    cli = robot_bridge.create_client(Empty, MAPSAVE_SERVICE)
    if not cli.wait_for_service(timeout_sec=2.0):
        raise HTTPException(status_code=503, detail="Map save service unavailable")
    req = Empty.Request()
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(robot_bridge, future, timeout_sec=5.0)
    if future.result() is not None:
        return {"status": "map_saved"}
    else:
        raise HTTPException(status_code=500, detail="Map save failed")

@app.get("/status")
async def status():
    if robot_bridge is None:
        raise HTTPException(status_code=500, detail="ROS bridge not initialized")
    data = {
        "chassis": robot_bridge.status_data,
        "odometry": robot_bridge.odom_data,
        "lidar": robot_bridge.lidar_data,
        "battery": robot_bridge.battery_data,
        "camera": "/camera/stream" if robot_bridge.camera_frame else None
    }
    return JSONResponse(data)

@app.get("/camera/stream")
async def camera_stream():
    if robot_bridge is None:
        raise HTTPException(status_code=500, detail="ROS bridge not initialized")
    async def gen():
        boundary = "--frame"
        while True:
            if robot_bridge.camera_frame is not None:
                yield (f"{boundary}\r\n"
                       f"Content-Type: image/jpeg\r\n\r\n").encode() + robot_bridge.camera_frame + b"\r\n"
            await asyncio.sleep(0.1)
    headers = {
        "Content-Type": "multipart/x-mixed-replace; boundary=--frame"
    }
    return StreamingResponse(gen(), headers=headers)

# --- Main Entrypoint ---
if __name__ == "__main__":
    uvicorn.run("main:app", host=HTTP_HOST, port=HTTP_PORT, reload=False)
