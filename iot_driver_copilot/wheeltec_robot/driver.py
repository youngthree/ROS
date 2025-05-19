import os
import json
import asyncio
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from fastapi import FastAPI, Request, Response
from fastapi.responses import StreamingResponse, JSONResponse
from std_msgs.msg import String
from sensor_msgs.msg import Image
from starlette.concurrency import run_in_threadpool
from typing import AsyncGenerator
import cv2
import numpy as np

# Environment Variables
ROBOT_IP = os.environ.get('ROBOT_IP', 'localhost')
ROS_NAMESPACE = os.environ.get('ROS_NAMESPACE', '')
HTTP_HOST = os.environ.get('HTTP_HOST', '0.0.0.0')
HTTP_PORT = int(os.environ.get('HTTP_PORT', '8000'))
CAMERA_TOPIC = os.environ.get('CAMERA_TOPIC', '/camera/image_raw')
CMD_VEL_TOPIC = os.environ.get('CMD_VEL_TOPIC', '/cmd_vel')
ROS_DOMAIN_ID = os.environ.get('ROS_DOMAIN_ID', None)
if ROS_DOMAIN_ID is not None:
    os.environ['ROS_DOMAIN_ID'] = ROS_DOMAIN_ID

# Robot command parameters (tunable)
LINEAR_VEL = float(os.environ.get('LINEAR_VEL', '0.2'))
ANGULAR_VEL = float(os.environ.get('ANGULAR_VEL', '0.7'))

# FastAPI App
app = FastAPI()

# ROS 2 Node Initialization - Singleton pattern for rclpy context
class Ros2Interface(Node):
    def __init__(self):
        super().__init__('wheeltec_http_driver')
        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.latest_image = None
        self.image_subscription = self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.image_callback,
            10
        )

    def image_callback(self, msg: Image):
        # Convert ROS2 Image msg to OpenCV image
        try:
            img_arr = np.frombuffer(msg.data, dtype=np.uint8)
            if msg.encoding == 'rgb8':
                shape = (msg.height, msg.width, 3)
                self.latest_image = img_arr.reshape(shape)
            elif msg.encoding == 'bgr8':
                shape = (msg.height, msg.width, 3)
                self.latest_image = img_arr.reshape(shape)
            elif msg.encoding == 'mono8':
                shape = (msg.height, msg.width)
                self.latest_image = cv2.cvtColor(img_arr.reshape(shape), cv2.COLOR_GRAY2BGR)
            # Add more encodings as needed
        except Exception:
            self.latest_image = None

    def send_cmd(self, linear=0.0, angular=0.0):
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.cmd_pub.publish(twist)

rclpy.init()
ros2_interface = Ros2Interface()

@app.on_event("shutdown")
def shutdown_event():
    ros2_interface.destroy_node()
    rclpy.shutdown()

# Command Endpoints
@app.post('/move/forward')
async def move_forward():
    await run_in_threadpool(ros2_interface.send_cmd, LINEAR_VEL, 0.0)
    return JSONResponse(content={"status": "ok", "action": "move_forward"})

@app.post('/move/backward')
async def move_backward():
    await run_in_threadpool(ros2_interface.send_cmd, -LINEAR_VEL, 0.0)
    return JSONResponse(content={"status": "ok", "action": "move_backward"})

@app.post('/turn/left')
async def turn_left():
    await run_in_threadpool(ros2_interface.send_cmd, 0.0, ANGULAR_VEL)
    return JSONResponse(content={"status": "ok", "action": "turn_left"})

@app.post('/turn/right')
async def turn_right():
    await run_in_threadpool(ros2_interface.send_cmd, 0.0, -ANGULAR_VEL)
    return JSONResponse(content={"status": "ok", "action": "turn_right"})

@app.post('/stop')
async def stop():
    await run_in_threadpool(ros2_interface.send_cmd, 0.0, 0.0)
    return JSONResponse(content={"status": "ok", "action": "stop"})

# MJPEG Streaming endpoint for camera
async def mjpeg_generator() -> AsyncGenerator[bytes, None]:
    boundary = "--frame"
    while True:
        img = ros2_interface.latest_image
        if img is not None:
            ret, jpg = cv2.imencode('.jpg', img)
            if ret:
                frame = jpg.tobytes()
                yield (
                    b"%s\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % (boundary.encode(), len(frame))
                    + frame + b"\r\n"
                )
        await asyncio.sleep(0.05)  # ~20 FPS

@app.get('/camera/stream')
async def camera_stream():
    headers = {
        "Content-Type": "multipart/x-mixed-replace; boundary=frame"
    }
    return StreamingResponse(mjpeg_generator(), headers=headers)

if __name__ == '__main__':
    import uvicorn
    uvicorn.run("main:app", host=HTTP_HOST, port=HTTP_PORT, reload=False)