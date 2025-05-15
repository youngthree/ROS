#include <cstdlib>
#include <iostream>
#include <thread>
#include <vector>
#include <sstream>
#include <string>
#include <map>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <cstring>
#include <chrono>
#include <algorithm>
#include <atomic>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>

// ================== CONFIGURATION =========================

std::string getenv_str(const char* env, const std::string& def = "") {
    const char* val = std::getenv(env);
    return val ? std::string(val) : def;
}

#define SERVER_HOST getenv_str("HTTP_SERVER_HOST", "0.0.0.0")
#define SERVER_PORT std::stoi(getenv_str("HTTP_SERVER_PORT", "8080"))
#define ROS_UDP_IP getenv_str("ROS_UDP_IP", "127.0.0.1")
#define ROS_UDP_PORT std::stoi(getenv_str("ROS_UDP_PORT", "15000"))
#define VIDEO_RTSP_URL getenv_str("VIDEO_RTSP_URL", "rtsp://127.0.0.1:8554/live")

// For POST /move
#define ROS_UDP_MOVE_PORT std::stoi(getenv_str("ROS_UDP_MOVE_PORT", "15001"))

// ================= SIMPLE HTTP SERVER =====================

struct HTTPRequest {
    std::string method;
    std::string path;
    std::map<std::string, std::string> headers;
    std::string body;
};

bool parse_http_request(const std::string& req, HTTPRequest& r) {
    std::istringstream ss(req);
    std::string line;
    if (!std::getline(ss, line)) return false;
    std::istringstream reqline(line);
    if (!(reqline >> r.method)) return false;
    if (!(reqline >> r.path)) return false;
    // Skip HTTP version
    std::getline(reqline, line);

    // Headers
    while (std::getline(ss, line) && line != "\r") {
        auto pos = line.find(':');
        if (pos != std::string::npos) {
            std::string key = line.substr(0,pos);
            std::string val = line.substr(pos+1);
            // trim
            key.erase(std::remove_if(key.begin(), key.end(), ::isspace), key.end());
            val.erase(0, val.find_first_not_of(" \t\r\n"));
            val.erase(val.find_last_not_of(" \t\r\n") + 1);
            r.headers[key] = val;
        }
    }
    // Body
    std::getline(ss, r.body, '\0');
    return true;
}

void send_response(int client, const std::string& status, const std::map<std::string,std::string>& headers, const std::string& body) {
    std::ostringstream resp;
    resp << "HTTP/1.1 " << status << "\r\n";
    for (const auto& h : headers) {
        resp << h.first << ": " << h.second << "\r\n";
    }
    resp << "Content-Length: " << body.size() << "\r\n";
    resp << "\r\n";
    resp << body;
    std::string str = resp.str();
    send(client, str.data(), str.size(), 0);
}

void send_chunked_header(int client, const std::map<std::string,std::string>& headers) {
    std::ostringstream resp;
    resp << "HTTP/1.1 200 OK\r\n";
    for (const auto& h : headers) {
        resp << h.first << ": " << h.second << "\r\n";
    }
    resp << "Transfer-Encoding: chunked\r\n\r\n";
    std::string str = resp.str();
    send(client, str.data(), str.size(), 0);
}

void send_chunk(int client, const char* data, size_t len) {
    char chunk_size[32];
    int chunklen = snprintf(chunk_size, sizeof(chunk_size), "%zx\r\n", len);
    send(client, chunk_size, chunklen, 0);
    if (len > 0) send(client, data, len, 0);
    send(client, "\r\n", 2, 0);
}

// ================= ROS UDP SUBSCRIBER (SIMPLIFIED) =================

struct SensorSample {
    std::string leg_odom;
    std::string imu;
    std::string joint_states;
    std::string handle_state;
    std::string us_distance;
    std::chrono::steady_clock::time_point last_update;
};

class ROSUDPSensorListener {
    int sock;
    std::atomic<bool> running;
    std::thread th;
    std::mutex mtx;
    SensorSample sample;
public:
    ROSUDPSensorListener() : sock(-1), running(false) {}
    ~ROSUDPSensorListener() { stop(); }

    void start(const std::string& ip, int port) {
        stop();
        running = true;
        sock = socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in addr {};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = inet_addr(ip.c_str());
        addr.sin_port = htons(port);
        bind(sock, (sockaddr*)&addr, sizeof(addr));
        th = std::thread([this]{
            char buf[2048];
            while (running) {
                sockaddr_in from_addr;
                socklen_t from_len = sizeof(from_addr);
                int n = recvfrom(sock, buf, sizeof(buf)-1, 0, (sockaddr*)&from_addr, &from_len);
                if (n > 0) {
                    buf[n] = 0;
                    std::unique_lock<std::mutex> lk(mtx);
                    parse_and_update(buf, n);
                }
            }
        });
    }

    void stop() {
        running = false;
        if (sock != -1) close(sock);
        if (th.joinable()) th.join();
        sock = -1;
    }

    SensorSample get_sample() {
        std::lock_guard<std::mutex> lk(mtx);
        return sample;
    }

    void parse_and_update(const char* buf, int len) {
        // For demo: Expect JSON {"leg_odom":"...","imu":"...","joint_states":"...","handle_state":"...","us_distance":"..."}
        // If other format, adapt as needed
        std::string s(buf, len);
        auto findval = [&](const char* key) -> std::string {
            auto k = std::string("\"") + key + "\"";
            auto p = s.find(k);
            if (p == std::string::npos) return "";
            auto q = s.find(':', p + k.size());
            if (q == std::string::npos) return "";
            auto q2 = s.find_first_of(",}", q+1);
            if (q2 == std::string::npos) return "";
            std::string v = s.substr(q+1, q2-q-1);
            v.erase(std::remove(v.begin(), v.end(), '\"'), v.end());
            v.erase(std::remove(v.begin(), v.end(), ' '), v.end());
            return v;
        };
        sample.leg_odom = findval("leg_odom");
        sample.imu = findval("imu");
        sample.joint_states = findval("joint_states");
        sample.handle_state = findval("handle_state");
        sample.us_distance = findval("us_distance");
        sample.last_update = std::chrono::steady_clock::now();
    }
};

// =============== VIDEO (RTSP->HTTP MJPEG PROXY) ===============
// Note: For maximum compatibility, we use H264-over-RTSP as input. We simulate MJPEG (for browser use) by extracting JPEGs from the stream.
// Since no third-party commands, we only serve the RTSP frames as binary (chunked) for now.
// In a production setting, a H264 parser and JPEG encoder would be needed.
// Here, we just open the RTSP as a TCP stream and forward data as chunked bytes.

#include <sys/types.h>
#include <netdb.h>

int open_tcp(const std::string& host, int port) {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return -1;
    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (inet_aton(host.c_str(), &addr.sin_addr) == 0) {
        close(sock);
        return -1;
    }
    if (connect(sock, (sockaddr*)&addr, sizeof(addr)) < 0) {
        close(sock);
        return -1;
    }
    return sock;
}

// Very basic RTSP client: Connect, setup, play, forward RTP packets (as raw) to the HTTP client
// This is a simplified version and does not handle all RTSP intricacies.
// In production, use a real parser/decoder.

class RTSPToHTTPStreamer {
    std::string rtsp_url;
public:
    RTSPToHTTPStreamer(const std::string& url) : rtsp_url(url) {}

    void stream_to_http(int client) {
        // Parse RTSP URL
        // Format: rtsp://host:port/path
        std::string host, path;
        int port = 554;
        if (rtsp_url.substr(0,7) == "rtsp://") {
            size_t p1 = 7;
            size_t p2 = rtsp_url.find('/', p1);
            size_t p_colon = rtsp_url.find(':', p1);
            if (p_colon != std::string::npos && p_colon < p2) {
                host = rtsp_url.substr(p1, p_colon-p1);
                port = std::stoi(rtsp_url.substr(p_colon+1, p2-p_colon-1));
            } else {
                host = rtsp_url.substr(p1, p2-p1);
            }
            path = rtsp_url.substr(p2);
        } else {
            send_response(client, "500 Internal Server Error", {{"Content-Type","text/plain"}}, "RTSP URL required\n");
            return;
        }
        int rtsp_sock = open_tcp(host, port);
        if (rtsp_sock < 0) {
            send_response(client, "502 Bad Gateway", {{"Content-Type","text/plain"}}, "Unable to connect to RTSP server\n");
            return;
        }
        // Send RTSP DESCRIBE/SETUP/PLAY (very minimal)
        auto send_rtsp = [&](const std::string& txt) {
            send(rtsp_sock, txt.data(), txt.size(), 0);
        };
        auto recv_rtsp = [&]() -> std::string {
            char buf[4096];
            int n = recv(rtsp_sock, buf, sizeof(buf)-1, 0);
            if (n <= 0) return "";
            buf[n] = 0;
            return std::string(buf, n);
        };
        static int cseq = 1;
        std::string session_id;
        // DESCRIBE
        std::ostringstream ss;
        ss << "DESCRIBE " << path << " RTSP/1.0\r\nCSeq: " << cseq++ << "\r\nAccept: application/sdp\r\n\r\n";
        send_rtsp(ss.str());
        recv_rtsp();
        // SETUP
        ss.str(""); ss.clear();
        ss << "SETUP " << path << "/trackID=1 RTSP/1.0\r\nCSeq: " << cseq++ << "\r\nTransport: RTP/AVP/TCP;unicast;interleaved=0-1\r\n\r\n";
        send_rtsp(ss.str());
        std::string setup_resp = recv_rtsp();
        // Find Session
        auto sid = setup_resp.find("Session:");
        if (sid != std::string::npos) {
            size_t sid_end = setup_resp.find("\r\n", sid);
            session_id = setup_resp.substr(sid+8, sid_end-sid-8);
            session_id.erase(std::remove(session_id.begin(), session_id.end(), ' '), session_id.end());
        }
        // PLAY
        ss.str(""); ss.clear();
        ss << "PLAY " << path << " RTSP/1.0\r\nCSeq: " << cseq++ << "\r\nSession: " << session_id << "\r\nRange: npt=0.000-\r\n\r\n";
        send_rtsp(ss.str());
        recv_rtsp();

        // HTTP response header
        std::map<std::string,std::string> headers = {
            {"Content-Type", "application/octet-stream"},
            {"Cache-Control", "no-cache"}
        };
        send_chunked_header(client, headers);

        // Forward data (raw interleaved RTP packets as binary chunks)
        char buf[4096];
        while (1) {
            int n = recv(rtsp_sock, buf, sizeof(buf), 0);
            if (n <= 0) break;
            send_chunk(client, buf, n);
        }
        // End chunked encoding
        send_chunk(client, "", 0);
        close(rtsp_sock);
    }
};

// =============== MOVE COMMAND SENDER (UDP) ==================

bool send_move_cmd(const std::string& cmd, const std::string& ip, int port) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) return false;
    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_aton(ip.c_str(), &addr.sin_addr);
    int n = sendto(sock, cmd.data(), cmd.size(), 0, (sockaddr*)&addr, sizeof(addr));
    close(sock);
    return n == (int)cmd.size();
}

// ================ MAIN HTTP HANDLER =========================

void handle_client(int client, ROSUDPSensorListener& ros_udp_listener) {
    char reqbuf[8192];
    int n = recv(client, reqbuf, sizeof(reqbuf)-1, 0);
    if (n <= 0) { close(client); return; }
    reqbuf[n] = 0;
    HTTPRequest req;
    if (!parse_http_request(reqbuf, req)) {
        send_response(client, "400 Bad Request", {{"Content-Type","text/plain"}}, "Malformed request.\n");
        close(client); return;
    }

    if (req.method == "GET" && req.path == "/sensors") {
        auto sample = ros_udp_listener.get_sample();
        std::ostringstream body;
        body << "{";
        body << "\"leg_odom\":\"" << sample.leg_odom << "\",";
        body << "\"imu\":\"" << sample.imu << "\",";
        body << "\"joint_states\":\"" << sample.joint_states << "\",";
        body << "\"handle_state\":\"" << sample.handle_state << "\",";
        body << "\"us_distance\":\"" << sample.us_distance << "\"";
        body << "}";
        send_response(client, "200 OK", {{"Content-Type","application/json"}}, body.str());
        close(client); return;

    } else if (req.method == "GET" && req.path == "/video") {
        RTSPToHTTPStreamer streamer(VIDEO_RTSP_URL);
        streamer.stream_to_http(client);
        close(client); return;

    } else if (req.method == "POST" && req.path == "/move") {
        // Assume JSON payload: {"linear":x,"angular":y} or {"cmd":"..."}
        std::string cmd = req.body.empty() ? "{}" : req.body;
        if (send_move_cmd(cmd, ROS_UDP_IP, ROS_UDP_MOVE_PORT)) {
            send_response(client, "200 OK", {{"Content-Type","application/json"}}, "{\"status\":\"ok\"}");
        } else {
            send_response(client, "500 Internal Server Error", {{"Content-Type","application/json"}}, "{\"status\":\"fail\"}");
        }
        close(client); return;
    } else {
        send_response(client, "404 Not Found", {{"Content-Type","text/plain"}}, "Unknown endpoint.\n");
        close(client); return;
    }
}

// ===================== MAIN ================================

int main() {
    // Start UDP sensor listener
    ROSUDPSensorListener ros_udp_listener;
    ros_udp_listener.start(ROS_UDP_IP, ROS_UDP_PORT);

    // HTTP Server
    int server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (server_sock < 0) { perror("socket"); return 1; }
    int opt = 1;
    setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(SERVER_PORT);
    addr.sin_addr.s_addr = inet_addr(SERVER_HOST.c_str());
    if (bind(server_sock, (sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind"); return 1;
    }
    if (listen(server_sock, 16) < 0) { perror("listen"); return 1; }
    std::cout << "HTTP Server listening on " << SERVER_HOST << ":" << SERVER_PORT << std::endl;

    while (true) {
        sockaddr_in client_addr {};
        socklen_t client_len = sizeof(client_addr);
        int client = accept(server_sock, (sockaddr*)&client_addr, &client_len);
        if (client < 0) continue;
        std::thread([client, &ros_udp_listener]{
            handle_client(client, ros_udp_listener);
        }).detach();
    }
    ros_udp_listener.stop();
    close(server_sock);
    return 0;
}