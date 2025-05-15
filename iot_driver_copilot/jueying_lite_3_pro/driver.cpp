#include <iostream>
#include <cstdlib>
#include <string>
#include <sstream>
#include <vector>
#include <thread>
#include <mutex>
#include <map>
#include <condition_variable>
#include <cstring>
#include <chrono>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <jsoncpp/json/json.h>

// ========== ENVIRONMENT VARIABLES ==========

std::string get_env(const char* var, const std::string& def) {
    const char* val = std::getenv(var);
    return val ? std::string(val) : def;
}

// Device config from environment
const std::string ROS_MASTER_IP   = get_env("DEVICE_IP", "127.0.0.1");
const int         ROS_MASTER_PORT = std::stoi(get_env("ROS_MASTER_PORT", "11311"));
const std::string UDP_SENSOR_IP   = get_env("UDP_SENSOR_IP", "127.0.0.1");
const int         UDP_SENSOR_PORT = std::stoi(get_env("UDP_SENSOR_PORT", "9500"));

// HTTP Server config
const std::string HTTP_HOST = get_env("SERVER_HOST", "0.0.0.0");
const int         HTTP_PORT = std::stoi(get_env("SERVER_PORT", "8080"));

// ========== UDP SENSOR DATA RECEIVER ==========

struct SensorData {
    std::string odometry;
    std::string imu;
    std::string joints;
    std::string ultrasound;
    std::string handle;
    std::string leg_odometry;
    std::string people_tracking;
    std::string pointcloud;
    std::string gridmap;
    std::mutex mtx;
} g_sensor_data;

void udp_sensor_listener() {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) return;

    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(UDP_SENSOR_IP.c_str());
    addr.sin_port = htons(UDP_SENSOR_PORT);

    if (bind(sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(sockfd);
        return;
    }

    char buffer[8192];
    while (true) {
        ssize_t len = recvfrom(sockfd, buffer, sizeof(buffer)-1, 0, NULL, NULL);
        if (len > 0) {
            buffer[len] = '\0';
            // Example: Expecting JSON-encoded UDP packets per data point
            Json::Value root;
            Json::Reader reader;
            if (reader.parse(buffer, root) && root.isObject()) {
                std::lock_guard<std::mutex> lock(g_sensor_data.mtx);
                if (root.isMember("odometry"))       g_sensor_data.odometry = root["odometry"].toStyledString();
                if (root.isMember("imu"))            g_sensor_data.imu = root["imu"].toStyledString();
                if (root.isMember("joints"))         g_sensor_data.joints = root["joints"].toStyledString();
                if (root.isMember("ultrasound"))     g_sensor_data.ultrasound = root["ultrasound"].toStyledString();
                if (root.isMember("handle"))         g_sensor_data.handle = root["handle"].toStyledString();
                if (root.isMember("leg_odometry"))   g_sensor_data.leg_odometry = root["leg_odometry"].toStyledString();
                if (root.isMember("people_tracking"))g_sensor_data.people_tracking = root["people_tracking"].toStyledString();
                if (root.isMember("pointcloud"))     g_sensor_data.pointcloud = root["pointcloud"].toStyledString();
                if (root.isMember("gridmap"))        g_sensor_data.gridmap = root["gridmap"].toStyledString();
            }
        }
    }
    close(sockfd);
}

// ========== HTTP SERVER UTILITIES ==========

struct HttpRequest {
    std::string method;
    std::string path;
    std::string body;
    std::map<std::string, std::string> headers;
};

struct HttpResponse {
    int status;
    std::string content_type;
    std::string body;
    HttpResponse(): status(200), content_type("application/json"), body("{}") {}
};

std::vector<std::string> split(const std::string& s, char delim) {
    std::vector<std::string> v;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) v.push_back(item);
    return v;
}

HttpRequest parse_http_request(const std::string& req) {
    HttpRequest r;
    std::istringstream ss(req);
    std::string line;
    std::getline(ss, line);
    std::vector<std::string> parts = split(line, ' ');
    if (parts.size() >= 2) {
        r.method = parts[0];
        r.path = parts[1];
    }
    while (std::getline(ss, line) && line != "\r") {
        if (line.empty() || line == "\r\n") break;
        auto pos = line.find(':');
        if (pos != std::string::npos) {
            std::string key = line.substr(0,pos);
            std::string val = line.substr(pos+1);
            if (!val.empty() && val[0] == ' ') val.erase(0,1);
            if (!val.empty() && val.back() == '\r') val.pop_back();
            r.headers[key] = val;
        }
    }
    if (r.headers.count("Content-Length")) {
        int cl = std::stoi(r.headers["Content-Length"]);
        std::string body(cl, '\0');
        ss.read(&body[0], cl);
        r.body = body;
    }
    return r;
}

void send_http_response(int client, const HttpResponse& resp) {
    std::ostringstream ss;
    ss << "HTTP/1.1 " << resp.status << " OK\r\n";
    ss << "Content-Type: " << resp.content_type << "\r\n";
    ss << "Access-Control-Allow-Origin: *\r\n";
    ss << "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n";
    ss << "Access-Control-Allow-Headers: Content-Type\r\n";
    ss << "Content-Length: " << resp.body.size() << "\r\n\r\n";
    ss << resp.body;
    std::string out = ss.str();
    send(client, out.data(), out.size(), 0);
}

void http_server_loop() {
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(HTTP_HOST.c_str());
    addr.sin_port = htons(HTTP_PORT);
    if (bind(server_fd, (sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "HTTP bind failed" << std::endl;
        exit(1);
    }
    listen(server_fd, 16);

    while (true) {
        int client = accept(server_fd, NULL, NULL);
        if (client < 0) continue;
        std::thread([client]() {
            char buffer[65536];
            ssize_t len = recv(client, buffer, sizeof(buffer)-1, 0);
            if (len <= 0) { close(client); return; }
            buffer[len] = '\0';
            std::string reqstr(buffer, len);
            HttpRequest req = parse_http_request(reqstr);

            HttpResponse resp;
            // OPTIONS preflight
            if (req.method == "OPTIONS") {
                resp.status = 204;
                resp.body = "";
                send_http_response(client, resp);
                close(client);
                return;
            }
            // Route
            if (req.method == "GET" && req.path == "/sdata") {
                // Aggregate latest sensor data as JSON
                Json::Value root;
                {
                    std::lock_guard<std::mutex> lock(g_sensor_data.mtx);
                    if (!g_sensor_data.odometry.empty())       root["odometry"] = Json::Value::null;
                    if (!g_sensor_data.imu.empty())            root["imu"] = Json::Value::null;
                    if (!g_sensor_data.joints.empty())         root["joints"] = Json::Value::null;
                    if (!g_sensor_data.ultrasound.empty())     root["ultrasound"] = Json::Value::null;
                    if (!g_sensor_data.handle.empty())         root["handle"] = Json::Value::null;
                    if (!g_sensor_data.leg_odometry.empty())   root["leg_odometry"] = Json::Value::null;
                    if (!g_sensor_data.people_tracking.empty())root["people_tracking"] = Json::Value::null;
                    if (!g_sensor_data.pointcloud.empty())     root["pointcloud"] = Json::Value::null;
                    if (!g_sensor_data.gridmap.empty())        root["gridmap"] = Json::Value::null;
                    if (!g_sensor_data.odometry.empty())       root["odometry"] = Json::Reader().parse(g_sensor_data.odometry, root["odometry"]) ? root["odometry"] : g_sensor_data.odometry;
                    if (!g_sensor_data.imu.empty())            root["imu"] = Json::Reader().parse(g_sensor_data.imu, root["imu"]) ? root["imu"] : g_sensor_data.imu;
                    if (!g_sensor_data.joints.empty())         root["joints"] = Json::Reader().parse(g_sensor_data.joints, root["joints"]) ? root["joints"] : g_sensor_data.joints;
                    if (!g_sensor_data.ultrasound.empty())     root["ultrasound"] = Json::Reader().parse(g_sensor_data.ultrasound, root["ultrasound"]) ? root["ultrasound"] : g_sensor_data.ultrasound;
                    if (!g_sensor_data.handle.empty())         root["handle"] = Json::Reader().parse(g_sensor_data.handle, root["handle"]) ? root["handle"] : g_sensor_data.handle;
                    if (!g_sensor_data.leg_odometry.empty())   root["leg_odometry"] = Json::Reader().parse(g_sensor_data.leg_odometry, root["leg_odometry"]) ? root["leg_odometry"] : g_sensor_data.leg_odometry;
                    if (!g_sensor_data.people_tracking.empty())root["people_tracking"] = Json::Reader().parse(g_sensor_data.people_tracking, root["people_tracking"]) ? root["people_tracking"] : g_sensor_data.people_tracking;
                    if (!g_sensor_data.pointcloud.empty())     root["pointcloud"] = Json::Reader().parse(g_sensor_data.pointcloud, root["pointcloud"]) ? root["pointcloud"] : g_sensor_data.pointcloud;
                    if (!g_sensor_data.gridmap.empty())        root["gridmap"] = Json::Reader().parse(g_sensor_data.gridmap, root["gridmap"]) ? root["gridmap"] : g_sensor_data.gridmap;
                }
                Json::FastWriter wr;
                resp.body = wr.write(root);
                send_http_response(client, resp);
            }
            else if (req.method == "POST" && req.path == "/vctrl") {
                // Parse JSON: { "linear": { "x": float, "y": float, "z": float }, "angular": { "x": float, "y": float, "z": float } }
                Json::Value root;
                Json::Reader reader;
                bool ok = reader.parse(req.body, root);
                if (!ok || !root.isMember("linear") || !root.isMember("angular")) {
                    resp.status = 400;
                    resp.body = "{\"error\":\"Malformed velocity command\"}";
                    send_http_response(client, resp);
                    close(client);
                    return;
                }
                // Here, send to robot via UDP as a JSON-encoded cmd_vel message
                Json::Value cmd;
                cmd["linear"] = root["linear"];
                cmd["angular"] = root["angular"];
                Json::FastWriter wr;
                std::string packet = wr.write(cmd);

                int sock = socket(AF_INET, SOCK_DGRAM, 0);
                sockaddr_in rob_addr;
                rob_addr.sin_family = AF_INET;
                rob_addr.sin_addr.s_addr = inet_addr(UDP_SENSOR_IP.c_str());
                rob_addr.sin_port = htons(UDP_SENSOR_PORT+1); // using +1 for command port, configurable if needed
                sendto(sock, packet.data(), packet.size(), 0, (sockaddr*)&rob_addr, sizeof(rob_addr));
                close(sock);

                resp.body = "{\"result\":\"ok\"}";
                send_http_response(client, resp);
            }
            else if (req.method == "POST" && req.path == "/navopt") {
                // Accept JSON: { "goal": {...}, "waypoints": [...] }
                // Forward to robot via UDP as a JSON-encoded command
                Json::Value root;
                Json::Reader reader;
                if (!reader.parse(req.body, root)) {
                    resp.status = 400;
                    resp.body = "{\"error\":\"Malformed navigation command\"}";
                    send_http_response(client, resp);
                    close(client);
                    return;
                }
                Json::FastWriter wr;
                std::string packet = wr.write(root);

                int sock = socket(AF_INET, SOCK_DGRAM, 0);
                sockaddr_in rob_addr;
                rob_addr.sin_family = AF_INET;
                rob_addr.sin_addr.s_addr = inet_addr(UDP_SENSOR_IP.c_str());
                rob_addr.sin_port = htons(UDP_SENSOR_PORT+2); // using +2 for nav command port
                sendto(sock, packet.data(), packet.size(), 0, (sockaddr*)&rob_addr, sizeof(rob_addr));
                close(sock);

                resp.body = "{\"result\":\"ok\"}";
                send_http_response(client, resp);
            }
            else {
                resp.status = 404;
                resp.content_type = "application/json";
                resp.body = "{\"error\":\"Not Found\"}";
                send_http_response(client, resp);
            }
            close(client);
        }).detach();
    }
    close(server_fd);
}

// ========== MAIN ==========

int main() {
    std::thread(udp_sensor_listener).detach();
    http_server_loop();
    return 0;
}