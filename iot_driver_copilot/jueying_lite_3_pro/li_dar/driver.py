import os
import asyncio
import json
from aiohttp import web
import aiohttp
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

ROS_HOST = os.environ.get("ROS_HOST", "localhost")
ROS_DOMAIN_ID = os.environ.get("ROS_DOMAIN_ID")
HTTP_SERVER_HOST = os.environ.get("HTTP_SERVER_HOST", "0.0.0.0")
HTTP_SERVER_PORT = int(os.environ.get("HTTP_SERVER_PORT", "8000"))
LINEAR_VEL = float(os.environ.get("LINEAR_VEL", "0.3"))
ANGULAR_VEL = float(os.environ.get("ANGULAR_VEL", "0.5"))
ODOM_TIMEOUT = float(os.environ.get("ODOM_TIMEOUT", "2.0"))

if ROS_DOMAIN_ID is not None:
    os.environ["ROS_DOMAIN_ID"] = ROS_DOMAIN_ID

rclpy.init()

class CommandNode(Node):
    def __init__(self):
        super().__init__('lite3pro_driver_cmd')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.last_odom_msg = None
        self.odom_event = asyncio.Event()
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

    def odom_cb(self, msg):
        self.last_odom_msg = msg
        self.odom_event.set()

    async def wait_for_odom(self, timeout=2.0):
        self.odom_event.clear()
        try:
            await asyncio.wait_for(self.odom_event.wait(), timeout)
        except asyncio.TimeoutError:
            return None
        return self.last_odom_msg

    def send_cmd(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher.publish(msg)

node = CommandNode()

async def run_ros_spin():
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        await asyncio.sleep(0.01)

routes = web.RouteTableDef()

@routes.post('/move/forward')
async def move_forward(request):
    node.send_cmd(linear_x=LINEAR_VEL, angular_z=0.0)
    return web.json_response({"status": "moving forward"})

@routes.post('/move/backward')
async def move_backward(request):
    node.send_cmd(linear_x=-LINEAR_VEL, angular_z=0.0)
    return web.json_response({"status": "moving backward"})

@routes.post('/turn/left')
async def turn_left(request):
    node.send_cmd(linear_x=0.0, angular_z=ANGULAR_VEL)
    return web.json_response({"status": "turning left"})

@routes.post('/turn/right')
async def turn_right(request):
    node.send_cmd(linear_x=0.0, angular_z=-ANGULAR_VEL)
    return web.json_response({"status": "turning right"})

@routes.post('/stop')
async def stop(request):
    node.send_cmd(linear_x=0.0, angular_z=0.0)
    return web.json_response({"status": "stopped"})

def odom_to_dict(msg):
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

@routes.get('/odom')
async def get_odom(request):
    msg = await node.wait_for_odom(timeout=ODOM_TIMEOUT)
    if msg is None:
        return web.json_response({"error": "odom timeout"}, status=504)
    return web.json_response(odom_to_dict(msg))

app = web.Application()
app.add_routes(routes)

async def main():
    loop = asyncio.get_running_loop()
    ros_task = loop.create_task(run_ros_spin())
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, HTTP_SERVER_HOST, HTTP_SERVER_PORT)
    await site.start()
    try:
        while True:
            await asyncio.sleep(3600)
    except (asyncio.CancelledError, KeyboardInterrupt):
        pass
    finally:
        await runner.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())