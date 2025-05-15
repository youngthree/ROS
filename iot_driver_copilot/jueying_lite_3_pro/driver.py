import os
import io
import json
import asyncio
import threading
from typing import Optional
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

import cv2
import numpy as np

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# --- Environment Variables ---
DEVICE_IP = os.environ.get('DEVICE_IP')
ROS_MASTER_URI = os.environ.get('ROS_MASTER_URI')  # e.g. 'http://192.168.1.10:11311'
SERVER_HOST = os.environ.get('SERVER_HOST', '0.0.0.0')
SERVER_PORT = int(os.environ.get('SERVER_PORT', '8000'))
RTSP_URL = os.environ.get('RTSP_URL')  # e.g. 'rtsp://192.168.1.10/stream'
ROS_NAMESPACE = os.environ.get('ROS_NAMESPACE', '')
ROS_CMD_VEL_TOPIC = os.environ.get('ROS_CMD_VEL_TOPIC', 'cmd_vel')
ROS_STATUS_TOPIC = os.environ.get('ROS_STATUS_TOPIC', 'robot_status')

# --- ROS Setup (run in another thread to avoid blocking HTTP server) ---
rospy_inited = False

def ros_init():
    global rospy_inited
    if not rospy_inited:
        rospy.init_node('jueying_lite3pro_driver', anonymous=True, disable_signals=True)
        rospy_inited = True

def ros_publish_cmd_vel(data):
    ros_init()
    pub = rospy.Publisher(ROS_CMD_VEL_TOPIC, Twist, queue_size=1)
    twist = Twist()
    twist.linear.x = data.get('linear', {}).get('x', 0)
    twist.linear.y = data.get('linear', {}).get('y', 0)
    twist.linear.z = data.get('linear', {}).get('z', 0)
    twist.angular.x = data.get('angular', {}).get('x', 0)
    twist.angular.y = data.get('angular', {}).get('y', 0)
    twist.angular.z = data.get('angular', {}).get('z', 0)
    pub.publish(twist)

# --- ROS Status Subscription ---
latest_status = {}

def ros_status_callback(msg):
    global latest_status
    latest_status = json.loads(msg.data)

def ros_subscribe_status():
    ros_init()
    rospy.Subscriber(ROS_STATUS_TOPIC, String, ros_status_callback)

status_thread = threading.Thread(target=ros_subscribe_status, daemon=True)
status_thread.start()

# --- ROS Script Control (simulate with a publisher) ---
def ros_manage_task(action, script_type):
    ros_init()
    pub = rospy.Publisher('script_control', String, queue_size=1)
    payload = {'action': action, 'script_type': script_type}
    pub.publish(json.dumps(payload))

# --- RTSP to HTTP JPEG MJPEG Streamer ---
class RTSPStreamer(threading.Thread):
    def __init__(self, rtsp_url):
        super().__init__()
        self.rtsp_url = rtsp_url
        self.cap = None
        self.latest_frame = None
        self.running = True
        self.lock = threading.Lock()
        self.daemon = True

    def run(self):
        self.cap = cv2.VideoCapture(self.rtsp_url)
        while self.running and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.latest_frame = frame
            else:
                break
        if self.cap:
            self.cap.release()

    def get_jpeg(self):
        with self.lock:
            if self.latest_frame is not None:
                ret, jpeg = cv2.imencode('.jpg', self.latest_frame)
                if ret:
                    return jpeg.tobytes()
            return None

    def stop(self):
        self.running = False

rtsp_streamer = None
if RTSP_URL:
    rtsp_streamer = RTSPStreamer(RTSP_URL)
    rtsp_streamer.start()

# --- HTTP Server ---
class RequestHandler(BaseHTTPRequestHandler):
    def _set_headers(self, status=200, mime='application/json'):
        self.send_response(status)
        self.send_header('Content-type', mime)
        self.end_headers()

    def do_GET(self):
        if self.path == '/status':
            self._set_headers()
            self.wfile.write(json.dumps(latest_status).encode())
        elif self.path == '/video':
            if not rtsp_streamer:
                self._set_headers(404)
                self.wfile.write(b'{"error":"RTSP streaming not configured"}')
                return
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    frame = rtsp_streamer.get_jpeg()
                    if frame is not None:
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                    else:
                        self.wfile.write(b'--frame\r\n\r\n')
                    self.wfile.flush()
                    # 20 fps
                    threading.Event().wait(0.05)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self._set_headers(404)
            self.wfile.write(b'{"error":"Not Found"}')

    def do_POST(self):
        content_length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(content_length) if content_length > 0 else b''
        response = {}
        if self.path == '/move':
            try:
                data = json.loads(body.decode())
                ros_publish_cmd_vel(data)
                self._set_headers()
                response = {"status": "sent", "data": data}
            except Exception as e:
                self._set_headers(400)
                response = {"error": str(e)}
        elif self.path == '/task':
            try:
                data = json.loads(body.decode())
                action = data.get('action')
                script_type = data.get('script_type')
                if not action or not script_type:
                    raise ValueError('Missing action or script_type')
                ros_manage_task(action, script_type)
                self._set_headers()
                response = {"status": "task action sent", "action": action, "script_type": script_type}
            except Exception as e:
                self._set_headers(400)
                response = {"error": str(e)}
        else:
            self._set_headers(404)
            response = {"error": "Not Found"}
        self.wfile.write(json.dumps(response).encode())

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    pass

def run_server():
    server_address = (SERVER_HOST, SERVER_PORT)
    httpd = ThreadedHTTPServer(server_address, RequestHandler)
    print(f'Serving HTTP on {SERVER_HOST}:{SERVER_PORT}...')
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    if rtsp_streamer:
        rtsp_streamer.stop()
    httpd.server_close()

if __name__ == '__main__':
    run_server()