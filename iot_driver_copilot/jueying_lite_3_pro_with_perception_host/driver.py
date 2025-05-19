import os
import json
from flask import Flask, request, jsonify, Response
import threading
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

# Load configuration from environment variables
ROS_MASTER_URI = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
ROS_HOSTNAME = os.environ.get('ROS_HOSTNAME', 'localhost')
HTTP_SERVER_HOST = os.environ.get('HTTP_SERVER_HOST', '0.0.0.0')
HTTP_SERVER_PORT = int(os.environ.get('HTTP_SERVER_PORT', '8080'))

# ROS node initialization is done in a thread-safe way
roscore_started = threading.Event()
odom_data_lock = threading.Lock()
last_odom = {}

def ros_spin():
    import time
    rospy.init_node('jueying_http_driver', anonymous=True, disable_signals=True)
    roscore_started.set()
    rospy.spin()

def odom_callback(msg):
    global last_odom
    with odom_data_lock:
        last_odom = {
            'header': {
                'stamp': msg.header.stamp.to_sec(),
                'frame_id': msg.header.frame_id
            },
            'child_frame_id': msg.child_frame_id,
            'pose': {
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
                'covariance': list(msg.pose.covariance)
            },
            'twist': {
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
                },
                'covariance': list(msg.twist.covariance)
            }
        }

def start_ros():
    threading.Thread(target=ros_spin, daemon=True).start()
    roscore_started.wait()
    rospy.Subscriber('/leg_odom', Odometry, odom_callback)

start_ros()

app = Flask(__name__)

# Helper: create or reuse publisher (threadsafe, single per topic)
class PublisherPool:
    def __init__(self):
        self.pubs = {}
        self.lock = threading.Lock()
    def get(self, topic, msg_type, queue_size=1):
        with self.lock:
            if topic not in self.pubs:
                self.pubs[topic] = rospy.Publisher(topic, msg_type, queue_size=queue_size)
            return self.pubs[topic]
publisher_pool = PublisherPool()

# API: move/forward
@app.route('/move/forward', methods=['POST'])
def move_forward():
    pub = publisher_pool.get('/cmd_vel', Twist)
    twist = Twist()
    twist.linear.x = 0.3
    twist.angular.z = 0.0
    pub.publish(twist)
    return jsonify({'status': 'moving forward'})

# API: move/backward
@app.route('/move/backward', methods=['POST'])
def move_backward():
    pub = publisher_pool.get('/cmd_vel', Twist)
    twist = Twist()
    twist.linear.x = -0.3
    twist.angular.z = 0.0
    pub.publish(twist)
    return jsonify({'status': 'moving backward'})

# API: turn/left
@app.route('/turn/left', methods=['POST'])
def turn_left():
    pub = publisher_pool.get('/cmd_vel', Twist)
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.6
    pub.publish(twist)
    return jsonify({'status': 'turning left'})

# API: turn/right
@app.route('/turn/right', methods=['POST'])
def turn_right():
    pub = publisher_pool.get('/cmd_vel', Twist)
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = -0.6
    pub.publish(twist)
    return jsonify({'status': 'turning right'})

# API: stop
@app.route('/stop', methods=['POST'])
def stop():
    pub = publisher_pool.get('/cmd_vel', Twist)
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)
    return jsonify({'status': 'stopped'})

# API: odom
@app.route('/odom', methods=['GET'])
def odom():
    with odom_data_lock:
        if last_odom:
            return Response(json.dumps(last_odom), mimetype='application/json')
        else:
            return jsonify({'error': 'No odometry data received yet.'}), 503

if __name__ == '__main__':
    app.run(host=HTTP_SERVER_HOST, port=HTTP_SERVER_PORT, threaded=True)