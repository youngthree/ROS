import os
import io
import json
import asyncio
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from urllib.parse import urlparse, parse_qs
import socket

import cv2
import numpy as np

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Imu, JointState

# Environment Variables
DEVICE_IP = os.environ.get('DEVICE_IP', '127.0.0.1')
ROS_MASTER_URI = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
ROS_HOSTNAME = os.environ.get('ROS_HOSTNAME', 'localhost')
RTSP_URL = os.environ.get('RTSP_URL', f'rtsp://{DEVICE_IP}/live/stream')
SERVER_HOST = os.environ.get('SERVER_HOST', '0.0.0.0')
SERVER_PORT = int(os.environ.get('SERVER_PORT', '8080'))

# ROS Node Initialization
os.environ['ROS_MASTER_URI'] = ROS_MASTER_URI
os.environ['ROS_HOSTNAME'] = ROS_HOSTNAME
if not rospy.core.is_initialized():
    rospy.init_node('lite3pro_http_driver', anonymous=True, disable_signals=True)

# ROS Data Holders
robot_status = {
    "localization": {},
    "navigation": {},
    "imu": {},
    "joint_states": {},
    "odom": {},
    "odom2": {},
    "handle_state": {},
    "ultrasound_distance": {},
    "yolov8_detection": {},
    "map_files": []
}
ros_lock = threading.Lock()

def ros_listener():
    def odom_cb(msg):
        with ros_lock:
            robot_status['odom'] = {
                "pose": {
                    "x": msg.pose.pose.position.x,
                    "y": msg.pose.pose.position.y,
                    "z": msg.pose.pose.position.z
                },
                "orientation": {
                    "x": msg.pose.pose.orientation.x,
                    "y": msg.pose.pose.orientation.y,
                    "z": msg.pose.pose.orientation.z,
                    "w": msg.pose.pose.orientation.w
                },
                "twist": {
                    "linear": {
                        "x": msg.twist.twist.linear.x,
                        "y": msg.twist.twist.linear.y,
                        "z": msg.twist.twist.linear.z,
                    },
                    "angular": {
                        "x": msg.twist.twist.angular.x,
                        "y": msg.twist.twist.angular.y,
                        "z": msg.twist.twist.angular.z,
                    }
                }
            }

    def imu_cb(msg):
        with ros_lock:
            robot_status['imu'] = {
                "orientation": {
                    "x": msg.orientation.x,
                    "y": msg.orientation.y,
                    "z": msg.orientation.z,
                    "w": msg.orientation.w,
                },
                "angular_velocity": {
                    "x": msg.angular_velocity.x,
                    "y": msg.angular_velocity.y,
                    "z": msg.angular_velocity.z,
                },
                "linear_acceleration": {
                    "x": msg.linear_acceleration.x,
                    "y": msg.linear_acceleration.y,
                    "z": msg.linear_acceleration.z,
                }
            }

    def joint_states_cb(msg):
        with ros_lock:
            robot_status['joint_states'] = {
                "name": list(msg.name),
                "position": list(msg.position),
                "velocity": list(msg.velocity),
                "effort": list(msg.effort)
            }

    def yolov8_cb(msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            data = msg.data
        with ros_lock:
            robot_status['yolov8_detection'] = data

    rospy.Subscriber('/odom', Odometry, odom_cb)
    rospy.Subscriber('/imu', Imu, imu_cb)
    rospy.Subscriber('/joint_states', JointState, joint_states_cb)
    rospy.Subscriber('/yolov8_detection', String, yolov8_cb)
    # Add more subscribers as needed

ros_thread = threading.Thread(target=ros_listener, daemon=True)
ros_thread.start()

# Helper: RTSP to MJPEG generator
def mjpeg_stream_generator(rtsp_url):
    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        yield b''
        return
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            ret, jpeg = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            jpg_bytes = jpeg.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpg_bytes + b'\r\n')
    finally:
        cap.release()

# ROS Publishers for /move and /task
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
cmd_vel_corr_pub = rospy.Publisher('/cmd_vel_corrected', Twist, queue_size=2)
task_pub = rospy.Publisher('/manage_task', String, queue_size=2) # Custom string topic for demo

def send_cmd_vel(payload):
    msg = Twist()
    if "linear" in payload:
        msg.linear.x = payload["linear"].get("x", 0)
        msg.linear.y = payload["linear"].get("y", 0)
        msg.linear.z = payload["linear"].get("z", 0)
    if "angular" in payload:
        msg.angular.x = payload["angular"].get("x", 0)
        msg.angular.y = payload["angular"].get("y", 0)
        msg.angular.z = payload["angular"].get("z", 0)
    cmd_vel_pub.publish(msg)

def send_cmd_vel_corrected(payload):
    msg = Twist()
    if "linear" in payload:
        msg.linear.x = payload["linear"].get("x", 0)
        msg.linear.y = payload["linear"].get("y", 0)
        msg.linear.z = payload["linear"].get("z", 0)
    if "angular" in payload:
        msg.angular.x = payload["angular"].get("x", 0)
        msg.angular.y = payload["angular"].get("y", 0)
        msg.angular.z = payload["angular"].get("z", 0)
    cmd_vel_corr_pub.publish(msg)

def handle_task(payload):
    # Publish task management command as json string
    msg = String()
    msg.data = json.dumps(payload)
    task_pub.publish(msg)
    return True

# HTTP Server implementation
class Handler(BaseHTTPRequestHandler):
    def _send_json(self, data, code=200):
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode('utf-8'))

    def _send_text(self, data, code=200):
        self.send_response(code)
        self.send_header('Content-Type', 'text/plain')
        self.end_headers()
        self.wfile.write(data.encode('utf-8'))

    def do_GET(self):
        parsed_path = urlparse(self.path)
        if parsed_path.path == '/status':
            with ros_lock:
                status = dict(robot_status)
            self._send_json(status)
        elif parsed_path.path == '/stream':
            # Proxy RTSP as MJPEG HTTP stream
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            for frame in mjpeg_stream_generator(RTSP_URL):
                try:
                    self.wfile.write(frame)
                except (BrokenPipeError, ConnectionResetError):
                    break
        else:
            self._send_json({"error": "Not found"}, code=404)

    def do_POST(self):
        parsed_path = urlparse(self.path)
        content_length = int(self.headers.get('Content-Length', 0))
        if content_length == 0:
            self._send_json({"error": "No JSON payload"}, code=400)
            return
        data = self.rfile.read(content_length)
        try:
            payload = json.loads(data.decode('utf-8'))
        except Exception:
            self._send_json({"error": "Invalid JSON"}, code=400)
            return

        if parsed_path.path == '/move':
            # Accepts {"linear": {...}, "angular": {...}, "mode": "corrected" or None}
            if "mode" in payload and payload["mode"] == "corrected":
                send_cmd_vel_corrected(payload)
            else:
                send_cmd_vel(payload)
            self._send_json({"success": True})

        elif parsed_path.path == '/task':
            # Accepts {"action": "start"/"stop", "script": "SLAM"/"LiDAR"/"Navigation"}
            result = handle_task(payload)
            self._send_json({"success": result})
        else:
            self._send_json({"error": "Not found"}, code=404)

    def log_message(self, format, *args):
        pass # Silence default logging

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True

def run():
    server_address = (SERVER_HOST, SERVER_PORT)
    httpd = ThreadedHTTPServer(server_address, Handler)
    print(f"Jueying Lite3Pro HTTP Driver running at http://{SERVER_HOST}:{SERVER_PORT}/")
    print("Endpoints: /status (GET), /move (POST), /task (POST), /stream (GET/MJPEG)")
    httpd.serve_forever()

if __name__ == '__main__':
    run()