import os
import json
import asyncio
import aiohttp
import aiohttp.web
import socket
import struct

# Environment Variables
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
DEVICE_ROS_UDP_PORT = int(os.environ.get("DEVICE_ROS_UDP_PORT", "15000"))
DEVICE_RTSP_PORT = int(os.environ.get("DEVICE_RTSP_PORT", "554"))
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", "8080"))
HTTP_VIDEO_ROUTE = os.environ.get("HTTP_VIDEO_ROUTE", "/video")

# --- ROS/UDP Helpers ---
def parse_udp_packet(data):
    # This should be tuned for your ROS/UDP payloads and demo only for JSON payloads
    try:
        msg = json.loads(data.decode("utf-8"))
        return msg
    except Exception:
        return {"raw": data.hex()}

async def fetch_status_from_udp():
    # Listen for a single UDP packet for demo purposes
    loop = asyncio.get_event_loop()
    fut = loop.create_datagram_endpoint(
        lambda: UDPPacketReceiver(),
        local_addr=('0.0.0.0', DEVICE_ROS_UDP_PORT)
    )
    transport, protocol = await fut
    try:
        msg = await asyncio.wait_for(protocol.get_message(), timeout=2.5)
    except asyncio.TimeoutError:
        msg = {"error": "Timeout waiting for UDP data"}
    transport.close()
    return msg

class UDPPacketReceiver(asyncio.DatagramProtocol):
    def __init__(self):
        self.msg = None
        self.event = asyncio.Event()

    def datagram_received(self, data, addr):
        self.msg = parse_udp_packet(data)
        self.event.set()

    async def get_message(self):
        await self.event.wait()
        return self.msg

# --- RTSP to HTTP Streaming ---
async def rtsp_to_http_stream(request):
    RTSP_URL = f"rtsp://{DEVICE_IP}:{DEVICE_RTSP_PORT}/video"
    # Direct proxying of RTSP to HTTP is not natively supported by browsers.
    # We'll extract H264 data and serve it as multipart/x-mixed-replace for browser (e.g. via MJPEG or fragmented MP4).
    # For demo, we establish RTSP, extract H264 and mux into multipart stream.
    # This requires manual parsing of RTSP/RTP for demo purposes.
    # This is a simplified RTSP/RTP/H264 extractor (does not cover all cases!)

    reader, writer = await asyncio.open_connection(DEVICE_IP, DEVICE_RTSP_PORT)
    cseq = 1

    def send_rtsp(cmd):
        nonlocal cseq
        writer.write((cmd + f"\r\nCSeq: {cseq}\r\n\r\n").encode())
        cseq += 1

    # 1. OPTIONS
    send_rtsp(f"OPTIONS rtsp://{DEVICE_IP}:{DEVICE_RTSP_PORT}/video RTSP/1.0")
    await writer.drain()
    await reader.readline()  # RTSP/1.0 200 OK

    # 2. DESCRIBE
    send_rtsp(f"DESCRIBE rtsp://{DEVICE_IP}:{DEVICE_RTSP_PORT}/video RTSP/1.0")
    await writer.drain()
    while True:
        line = await reader.readline()
        if not line or line == b'\r\n':
            break

    # 3. SETUP (UDP - unsupported here, so use TCP interleaved)
    send_rtsp(f"SETUP rtsp://{DEVICE_IP}:{DEVICE_RTSP_PORT}/video/trackID=1 RTSP/1.0\r\nTransport: RTP/AVP/TCP;unicast;interleaved=0-1")
    await writer.drain()
    session_id = None
    while True:
        line = await reader.readline()
        if not line or line == b'\r\n':
            break
        if b"Session:" in line:
            session_id = line.decode().split("Session:")[1].strip()
    if not session_id:
        return aiohttp.web.Response(status=500, text="RTSP Session error")

    # 4. PLAY
    send_rtsp(f"PLAY rtsp://{DEVICE_IP}:{DEVICE_RTSP_PORT}/video RTSP/1.0\r\nSession: {session_id}")
    await writer.drain()
    while True:
        line = await reader.readline()
        if not line or line == b'\r\n':
            break

    # 5. Proxy RTP packets as MJPEG multipart/x-mixed-replace for browser
    boundary = "frame"
    response = aiohttp.web.StreamResponse(
        status=200,
        reason='OK',
        headers={
            "Content-Type": f"multipart/x-mixed-replace; boundary={boundary}"
        }
    )
    await response.prepare(request)

    try:
        while True:
            # RTP over RTSP/TCP: Each packet starts with $ (0x24), channel, length (2 bytes)
            dollar = await reader.readexactly(1)
            if dollar != b'$':
                continue
            channel = await reader.readexactly(1)
            plen = struct.unpack(">H", await reader.readexactly(2))[0]
            rtp = await reader.readexactly(plen)
            # RTP header is 12 bytes; H.264 payload follows
            payload = rtp[12:]
            # For demo, treat payload as JPEG frame (if MJPEG stream), else wrap as is.
            # Real MJPEG: JPEG magic bytes (0xFFD8...FFD9)
            # For demo, just act as if payload is JPEG
            await response.write(
                f"--{boundary}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(payload)}\r\n\r\n".encode() +
                payload +
                b"\r\n"
            )
            await response.drain()
    except Exception:
        pass
    finally:
        writer.close()
        await writer.wait_closed()
    return response

# --- REST API Handlers ---

async def handle_status(request):
    # Fetch status (localization, navigation, etc.) from UDP/ROS-topic
    status = await fetch_status_from_udp()
    return aiohttp.web.json_response(status)

async def handle_task(request):
    # Manage operational scripts (SLAM, LiDAR, navigation) via ROS/UDP or other protocol
    # For demo, just echo the request and simulate a success
    try:
        data = await request.json()
    except Exception:
        return aiohttp.web.json_response({"error": "Invalid JSON"}, status=400)
    action = data.get("action")
    script_type = data.get("script_type")
    # Here, you would send a ROS/UDP message or similar to the robot
    # Simulate success
    return aiohttp.web.json_response({"status": "success", "action": action, "script_type": script_type})

async def handle_move(request):
    # Send movement (cmd_vel, etc.) commands to the robot (for demo, UDP packet send)
    try:
        data = await request.json()
    except Exception:
        return aiohttp.web.json_response({"error": "Invalid JSON"}, status=400)
    # Send JSON as UDP packet to robot's ROS/UDP endpoint
    message = json.dumps(data).encode("utf-8")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(message, (DEVICE_IP, DEVICE_ROS_UDP_PORT))
        sock.close()
    except Exception as e:
        return aiohttp.web.json_response({"status": "error", "detail": str(e)}, status=500)
    return aiohttp.web.json_response({"status": "sent"})

# --- Main App ---
def main():
    app = aiohttp.web.Application()
    app.router.add_post("/task", handle_task)
    app.router.add_get("/status", handle_status)
    app.router.add_post("/move", handle_move)
    app.router.add_get(HTTP_VIDEO_ROUTE, rtsp_to_http_stream)
    aiohttp.web.run_app(app, host=SERVER_HOST, port=SERVER_PORT)

if __name__ == "__main__":
    main()