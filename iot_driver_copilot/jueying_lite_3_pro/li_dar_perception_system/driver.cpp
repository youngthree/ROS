#include <iostream>
#include <string>
#include <cstdlib>
#include <thread>
#include <sstream>
#include <fstream>
#include <vector>
#include <map>
#include <mutex>
#include <condition_variable>
#include <cstring>
#include <cstdio>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <jsoncpp/json/json.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <ros/service_client.h>

// ------------ ENV UTILS ---------------
std::string getenv_str(const char* name, const char* def = "") {
    const char* v = std::getenv(name);
    return v ? std::string(v) : std::string(def);
}

int getenv_int(const char* name, int def) {
    const char* v = std::getenv(name);
    return v ? atoi(v) : def;
}

// ------------ HTTP SERVER -------------
class HttpResponse {
public:
    int status;
    std::string content_type;
    std::string body;
    std::map<std::string, std::string> headers;

    HttpResponse(): status(200), content_type("application/json") {}

    std::string to_string() {
        std::ostringstream oss;
        oss << "HTTP/1.1 " << status << " "
            << (status==200 ? "OK" : (status==404 ? "Not Found" : "Error")) << "\r\n";
        oss << "Content-Type: " << content_type << "\r\n";
        oss << "Content-Length: " << body.size() << "\r\n";
        for (const auto& kv : headers) {
            oss << kv.first << ": " << kv.second << "\r\n";
        }
        oss << "Connection: close\r\n";
        oss << "\r\n";
        oss << body;
        return oss.str();
    }
};

class HttpRequest {
public:
    std::string method;
    std::string path;
    std::map<std::string, std::string> headers;
    std::string body;
};

bool parse_http_request(const std::string& raw, HttpRequest& req) {
    std::istringstream iss(raw);
    std::string line;
    if (!std::getline(iss, line)) return false;
    std::istringstream lss(line);
    lss >> req.method >> req.path;
    while (std::getline(iss, line) && line != "\r") {
        size_t p = line.find(':');
        if (p != std::string::npos) {
            std::string key = line.substr(0, p);
            std::string val = line.substr(p + 1);
            while (!val.empty() && (val[0] == ' ' || val[0] == '\t')) val.erase(0, 1);
            if (!val.empty() && val.back() == '\r') val.pop_back();
            req.headers[key] = val;
        }
    }
    std::ostringstream bodyoss;
    while (std::getline(iss, line)) bodyoss << line << "\n";
    req.body = bodyoss.str();
    if (!req.body.empty() && req.body.back() == '\n') req.body.pop_back();
    return true;
}

class SimpleHttpServer {
public:
    SimpleHttpServer(const std::string& host, int port)
        : host_(host), port_(port), running_(false) {}

    void start() {
        running_ = true;
        srv_thread_ = std::thread([this](){ this->run(); });
    }

    void stop() {
        running_ = false;
        close(server_fd_);
        if (srv_thread_.joinable()) srv_thread_.join();
    }

    typedef std::function<HttpResponse(const HttpRequest&)> HandlerFunc;
    void route(const std::string& method, const std::string& path, HandlerFunc h) {
        std::string key = method + " " + path;
        handlers_[key] = h;
    }

private:
    std::string host_;
    int port_;
    int server_fd_;
    std::thread srv_thread_;
    bool running_;
    std::map<std::string, HandlerFunc> handlers_;

    void run() {
        struct sockaddr_in address;
        int opt = 1;
        int addrlen = sizeof(address);

        if ((server_fd_ = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
            perror("socket failed");
            return;
        }
        if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
            perror("setsockopt");
            return;
        }
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = inet_addr(host_.c_str());
        address.sin_port = htons(port_);

        if (bind(server_fd_, (struct sockaddr *)&address, sizeof(address))<0) {
            perror("bind failed");
            return;
        }
        if (listen(server_fd_, 8) < 0) {
            perror("listen");
            return;
        }
        while (running_) {
            int new_socket = accept(server_fd_, (struct sockaddr *)&address, (socklen_t*)&addrlen);
            if (new_socket < 0) {
                if (!running_) break;
                perror("accept");
                continue;
            }
            std::thread([this, new_socket](){ this->handle_client(new_socket); }).detach();
        }
    }

    void handle_client(int sock) {
        char buffer[8192] = {0};
        ssize_t valread = read(sock, buffer, sizeof(buffer)-1);
        if (valread <= 0) { close(sock); return; }
        HttpRequest req;
        if (!parse_http_request(buffer, req)) {
            close(sock); return;
        }
        std::string key = req.method + " " + req.path;
        HttpResponse resp;
        if (handlers_.count(key)) {
            resp = handlers_[key](req);
        } else if (req.method == "GET" && req.path == "/video") {
            // Special handler: RTSP to HTTP MJPEG proxy
            handle_rtsp_to_http(sock);
            return;
        } else {
            resp.status = 404;
            resp.body = "{\"error\":\"not found\"}";
        }
        std::string out = resp.to_string();
        send(sock, out.c_str(), out.size(), 0);
        close(sock);
    }

    void handle_rtsp_to_http(int sock) {
        // RTSP MJPEG proxy (partial implementation: UDP/RTP transport, MJPEG only)
        // [For simplicity, only MJPEG over RTP/UDP, as supported by some robots/cameras.]
        // The client connects to /video, and this method streams HTTP multipart/x-mixed-replace
        std::string rtsp_url = getenv_str("ROBOT_RTSP_URL");
        std::string robot_ip = getenv_str("ROBOT_IP");
        int rtsp_port = getenv_int("ROBOT_RTSP_PORT", 554);
        // Compose RTSP URL if not given
        if (rtsp_url.empty() && !robot_ip.empty()) {
            rtsp_url = "rtsp://" + robot_ip + ":" + std::to_string(rtsp_port) + "/live";
        }
        // Minimal MJPEG-over-RTP UDP parser (for PoC only)
        // This will NOT work for H.264/5 without heavy parsing - only MJPEG RTP is attempted here.

        std::string header =
            "HTTP/1.1 200 OK\r\n"
            "Server: RobotDriver\r\n"
            "Content-Type: multipart/x-mixed-replace;boundary=frame\r\n"
            "Cache-Control: no-cache\r\n"
            "Connection: close\r\n\r\n";
        send(sock, header.c_str(), header.size(), 0);

        // Open UDP socket to receive RTP packets (assuming robot streams MJPEG over UDP - e.g., port 5600)
        int udp_port = getenv_int("ROBOT_UDP_VIDEO_PORT", 5600);
        int udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_sock < 0) return;
        struct sockaddr_in addr;
        memset((char *)&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(udp_port);
        if (bind(udp_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            close(udp_sock); return;
        }
        // RTP MJPEG parsing loop
        unsigned char pkt[65536];
        size_t jpeg_buf_sz = 512*1024;
        std::vector<unsigned char> jpeg_buf(jpeg_buf_sz);
        size_t jpeg_pos = 0;
        bool in_frame = false;
        fd_set fds;
        struct timeval tv;
        while (1) {
            FD_ZERO(&fds);
            FD_SET(udp_sock, &fds);
            tv.tv_sec = 0;
            tv.tv_usec = 200000;
            int sel = select(udp_sock+1, &fds, 0, 0, &tv);
            if (sel > 0 && FD_ISSET(udp_sock, &fds)) {
                ssize_t n = recvfrom(udp_sock, pkt, sizeof(pkt), 0, NULL, NULL);
                if (n < 12) continue;
                // RTP header: 12 bytes
                int payload_offset = 12;
                // RTP MJPEG: [RFC 2435]
                if ((pkt[1]&0x7F) == 26) { // MJPEG
                    // Parse marker bit for frame end
                    bool marker = pkt[1] & 0x80;
                    if (!in_frame) jpeg_pos = 0;
                    size_t plen = n - payload_offset;
                    if (jpeg_pos + plen < jpeg_buf.size()) {
                        memcpy(jpeg_buf.data()+jpeg_pos, pkt+payload_offset, plen);
                        jpeg_pos += plen;
                        in_frame = true;
                    }
                    if (marker && in_frame && jpeg_pos>0) {
                        // Emit frame
                        std::ostringstream oss;
                        oss << "--frame\r\n"
                            << "Content-Type: image/jpeg\r\n"
                            << "Content-Length: " << jpeg_pos << "\r\n\r\n";
                        send(sock, oss.str().c_str(), oss.str().size(), 0);
                        send(sock, (char*)jpeg_buf.data(), jpeg_pos, 0);
                        send(sock, "\r\n", 2, 0);
                        in_frame = false; jpeg_pos=0;
                    }
                }
            }
            // Socket closed by client?
            char test;
            if (recv(sock, &test, 1, MSG_PEEK|MSG_DONTWAIT) == 0) break;
        }
        close(udp_sock);
        close(sock);
    }
};

// --------- ROS CLIENT HANDLERS ------------
class RobotROSClient {
public:
    RobotROSClient(const std::string& master_uri, const std::string& ns):
        master_uri_(master_uri), ns_(ns), nh_(nullptr), twist_pub_(nullptr)
    {
        int argc = 0;
        char **argv = nullptr;
        ros::init(argc, argv, "robot_http_driver", ros::init_options::NoSigintHandler);
        nh_ = std::make_shared<ros::NodeHandle>();
        twist_pub_ = std::make_shared<ros::Publisher>(
            nh_->advertise<geometry_msgs::Twist>(ns_ + "/cmd_vel", 2)
        );
    }

    bool send_move(double lin_x, double lin_y, double lin_z, double ang_x, double ang_y, double ang_z) {
        if (!twist_pub_ || !twist_pub_->getNumSubscribers()) return false;
        geometry_msgs::Twist tw;
        tw.linear.x = lin_x;
        tw.linear.y = lin_y;
        tw.linear.z = lin_z;
        tw.angular.x = ang_x;
        tw.angular.y = ang_y;
        tw.angular.z = ang_z;
        twist_pub_->publish(tw);
        return true;
    }

    bool call_service(const std::string& service, const std::string& action) {
        // Only allow starting/stopping known services
        static std::map<std::string, std::string> service_map = {
            {"slam", "/start_slam"}, {"nav", "/start_nav"}, {"lslidar", "/start_lslidar"},
            {"record", "/location_record"}, {"gridmap", "/gridmap"}, {"savemap", "/save_map"}
        };
        if (service_map.count(service) == 0) return false;
        std_srvs::Empty srv;
        std::string srv_name = service_map[service];
        ros::ServiceClient client = nh_->serviceClient<std_srvs::Empty>(srv_name);
        if (action == "start") {
            return client.call(srv);
        } else if (action == "stop") {
            return client.call(srv);
        }
        return false;
    }

private:
    std::string master_uri_, ns_;
    std::shared_ptr<ros::NodeHandle> nh_;
    std::shared_ptr<ros::Publisher> twist_pub_;
};


// ------------------- MAIN ----------------------
int main(int argc, char** argv) {
    // Read env vars
    std::string host = getenv_str("HTTP_SERVER_HOST", "0.0.0.0");
    int port = getenv_int("HTTP_SERVER_PORT", 8080);

    std::string robot_ip = getenv_str("ROBOT_IP");
    std::string ros_master = getenv_str("ROS_MASTER_URI", "http://localhost:11311");
    std::string ros_ns = getenv_str("ROBOT_NAMESPACE", "");

    SimpleHttpServer server(host, port);
    RobotROSClient ros_client(ros_master, ros_ns);

    // /info
    server.route("GET", "/info", [](const HttpRequest& req) {
        Json::Value j;
        j["device_name"] = "Jueying Lite3 Pro/LiDAR Perception System";
        j["device_model"] = "Lite3 Pro";
        j["manufacturer"] = "DEEP Robotics";
        j["device_type"] = "Mobile Robot Platform";
        j["protocols"] = Json::arrayValue;
        j["protocols"].append("ROS");
        j["protocols"].append("UDP");
        j["protocols"].append("RTSP");
        HttpResponse resp;
        resp.content_type = "application/json";
        Json::FastWriter w;
        resp.body = w.write(j);
        return resp;
    });

    // /move
    server.route("POST", "/move",
        [&ros_client](const HttpRequest& req) {
            Json::Reader rdr;
            Json::Value j;
            HttpResponse resp;
            if (!rdr.parse(req.body, j)) {
                resp.status = 400;
                resp.body = "{\"error\":\"Invalid JSON\"}";
                return resp;
            }
            double lx = j.get("linear_x", 0.0).asDouble();
            double ly = j.get("linear_y", 0.0).asDouble();
            double lz = j.get("linear_z", 0.0).asDouble();
            double ax = j.get("angular_x", 0.0).asDouble();
            double ay = j.get("angular_y", 0.0).asDouble();
            double az = j.get("angular_z", 0.0).asDouble();
            bool ok = ros_client.send_move(lx, ly, lz, ax, ay, az);
            if (!ok) {
                resp.status = 400;
                resp.body = "{\"error\":\"Failed to send move\"}";
            } else {
                resp.body = "{\"status\":\"ok\"}";
            }
            return resp;
        });

    // /serv
    server.route("POST", "/serv",
        [&ros_client](const HttpRequest& req) {
            Json::Reader rdr;
            Json::Value j;
            HttpResponse resp;
            if (!rdr.parse(req.body, j)) {
                resp.status = 400;
                resp.body = "{\"error\":\"Invalid JSON\"}";
                return resp;
            }
            std::string action = j.get("action", "").asString();
            std::string service = j.get("service", "").asString();
            if (action.empty() || service.empty()) {
                resp.status = 400;
                resp.body = "{\"error\":\"Missing action or service\"}";
                return resp;
            }
            bool ok = ros_client.call_service(service, action);
            if (!ok) {
                resp.status = 400;
                resp.body = "{\"error\":\"Failed to call service\"}";
            } else {
                resp.body = "{\"status\":\"ok\"}";
            }
            return resp;
        });

    // /video handled in special handler in server

    std::cout << "Robot HTTP driver starting at " << host << ":" << port << std::endl;
    server.start();
    while (true) { std::this_thread::sleep_for(std::chrono::seconds(1)); }
    server.stop();
    return 0;
}