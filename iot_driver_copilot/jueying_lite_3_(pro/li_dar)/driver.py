import os
import asyncio
import json
from fastapi import FastAPI, Request, Response, HTTPException
from fastapi.responses import StreamingResponse, JSONResponse
from pydantic import BaseModel
import uvicorn

import aiohttp

# ROS bridge imports
import websockets
import base64

# Environment Variables
ROBOT_IP = os.environ.get("ROBOT_IP", "127.0.0.1")
ROSBRIDGE_PORT = int(os.environ.get("ROSBRIDGE_PORT", "9090"))
UDP_PORT = int(os.environ.get("UDP_PORT", "9000"))
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", "8080"))

ROSBRIDGE_URI = f"ws://{ROBOT_IP}:{ROSBRIDGE_PORT}"

app = FastAPI()

# ======== Models for API endpoints ========

class MoveCommand(BaseModel):
    linear_x: float
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0
    angular_z: float

class NavTask(BaseModel):
    waypoints: list  # list of [x, y, theta] or dicts, flexible for user

# ======== ROS Bridge Utilities ========

async def rosbridge_call(service, args):
    '''
    Generic ROS bridge call service (not used for this device, but for completeness)
    '''
    async with websockets.connect(ROSBRIDGE_URI) as ws:
        msg = {
            "op": "call_service",
            "service": service,
            "args": args,
            "id": "srv1"
        }
        await ws.send(json.dumps(msg))
        async for message in ws:
            rep = json.loads(message)
            if rep.get("id") == "srv1":
                return rep

async def rosbridge_publish(topic, msg_type, msg_data):
    async with websockets.connect(ROSBRIDGE_URI) as ws:
        msg = {
            "op": "publish",
            "topic": topic,
            "msg": msg_data
        }
        await ws.send(json.dumps(msg))
        # We don't expect a response for publish

async def rosbridge_subscribe_once(topic, msg_type):
    # Returns the first message received on this topic
    async with websockets.connect(ROSBRIDGE_URI) as ws:
        sub_msg = {
            "op": "subscribe",
            "topic": topic,
            "type": msg_type,
            "id": "sub1"
        }
        await ws.send(json.dumps(sub_msg))
        while True:
            rep = json.loads(await ws.recv())
            if rep.get("topic") == topic and "msg" in rep:
                return rep["msg"]

async def rosbridge_subscribe_multi(topics_types, timeout=1.2):
    '''
    topics_types: list of (topic, msg_type) tuples
    Collects the first message of each topic within timeout.
    '''
    results = {}
    async with websockets.connect(ROSBRIDGE_URI) as ws:
        for i, (topic, msg_type) in enumerate(topics_types):
            sub_msg = {
                "op": "subscribe",
                "topic": topic,
                "type": msg_type,
                "id": f"sub{i}"
            }
            await ws.send(json.dumps(sub_msg))

        async def collect_msgs():
            end_time = asyncio.get_event_loop().time() + timeout
            while len(results) < len(topics_types) and asyncio.get_event_loop().time() < end_time:
                try:
                    rep = json.loads(await asyncio.wait_for(ws.recv(), timeout=timeout))
                except:
                    break
                topic = rep.get("topic")
                if topic and "msg" in rep and topic not in results:
                    results[topic] = rep["msg"]
            return results

        return await collect_msgs()

# ======== UDP command for navigation ========

async def udp_send_command(ip, port, data: bytes):
    # Used for navigation multi-point tasks via UDP SimpleCMD if necessary
    loop = asyncio.get_event_loop()
    transport, protocol = await loop.create_datagram_endpoint(
        asyncio.DatagramProtocol, remote_addr=(ip, port)
    )
    transport.sendto(data)
    transport.close()

# ======== API Endpoints ========

@app.get("/status", response_class=JSONResponse)
async def status():
    """
    Retrieves the current sensor and state data from the robot.
    Includes: /leg_odom, /leg_odom2, /joint_states, /imu/data, /us_publisher/ultrasound_distance
    """
    topics_types = [
        ("/leg_odom", "nav_msgs/Odometry"),
        ("/leg_odom2", "nav_msgs/Odometry"),
        ("/joint_states", "sensor_msgs/JointState"),
        ("/imu/data", "sensor_msgs/Imu"),
        ("/us_publisher/ultrasound_distance", "std_msgs/Float32MultiArray")
    ]
    results = await rosbridge_subscribe_multi(topics_types)
    return JSONResponse(content=results)

@app.post("/move", response_class=JSONResponse)
async def move(cmd: MoveCommand):
    """
    Sends movement commands to the robot by writing to the /cmd_vel channel.
    """
    msg_data = {
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
    await rosbridge_publish("/cmd_vel", "geometry_msgs/Twist", msg_data)
    return {"status": "ok"}

@app.post("/nav", response_class=JSONResponse)
async def nav(task: NavTask):
    """
    Initiates a multi-point navigation task using the device's Task.py command.
    Users provide waypoints, which are relayed as a navigation command.
    """
    # Simulate Task.py invocation by publishing to a ROS topic, or UDP SimpleCMD
    # Let's assume we publish to a topic /task_nav with waypoints.
    # If not available, this needs device-side adaptation.
    msg_data = {
        "waypoints": task.waypoints
    }
    try:
        await rosbridge_publish("/task_nav", "std_msgs/String", {"data": json.dumps(msg_data)})
        return {"status": "ok"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

# ======== Main Entrypoint ========

if __name__ == "__main__":
    uvicorn.run("main:app", host=SERVER_HOST, port=SERVER_PORT, reload=False)