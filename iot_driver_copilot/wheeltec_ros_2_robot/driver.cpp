```cpp
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <sstream>
#include <unordered_map>
#include <vector>
#include <cstdio>
#include <chrono>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <regex>
#include <nlohmann/json.hpp>

// ROS2 headers
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

// Simple HTTP server (from scratch, single-threaded, minimal)
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

using json = nlohmann::json;

// --------- Environment Variables -----------
std::string get_env(const char *var, const std::string &defval) {
    const char *v = std::getenv(var);
    if (v) return std::string(v);
    return defval;
}

const std::string HTTP_HOST = get_env("DRIVER_HTTP_HOST", "0.0.0.0");
const int HTTP_PORT = std::stoi(get_env("DRIVER_HTTP_PORT", "8080"));

// ROS2 config
const std::string ROS2_NODE_NAME = get_env("DRIVER_ROS2_NODE_NAME", "device_shifu_driver");
const std::string ROS2_CMDVEL_TOPIC = get_env("DRIVER_ROS2_CMDVEL_TOPIC", "/cmd_vel");

// --------- ROS2 Control -----------
class ROS2Publisher {
public:
    ROS2Publisher() {
        // ROS2 spin in separate thread
        rclcpp::init(0, nullptr);
        node_ = rclcpp::Node::make_shared(ROS2_NODE_NAME);
        pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(ROS2_CMDVEL_TOPIC, 10);
        running_ = true;
        spin_thread_ = std::thread([this](){
            while (running_) {
                rclcpp::spin_some(node_);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
    }
    ~ROS2Publisher() {
        running_ = false;
        if (spin_thread_.joinable()) spin_thread_.join();
        rclcpp::shutdown();
    }

    void publishTwist(double linear_x, double linear_y, double angular_z) {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = linear_x;
        msg.linear.y = linear_y;
        msg.linear.z = 0.0;
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = angular_z;
        pub_->publish(msg);
    }
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    std::thread spin_thread_;
    std::atomic<bool> running_;
};

// --------- HTTP Server -----------
class HTTPServer {
public:
    HTTPServer(const std::string& host, int port, ROS2Publisher& ros2pub)
        : host_(host), port_(port), ros2pub_(ros2pub) {}

    void run() {
        int server_fd, new_socket;
        struct sockaddr_in address;
        int opt = 1;
        int addrlen = sizeof(address);

        if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
            perror("Socket failed");
            exit(EXIT_FAILURE);
        }
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
            perror("setsockopt");
            exit(EXIT_FAILURE);
        }
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = (host_ == "0.0.0.0") ? INADDR_ANY : inet_addr(host_.c_str());
        address.sin_port = htons(port_);

        if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
            perror("Bind failed");
            exit(EXIT_FAILURE);
        }
        if (listen(server_fd, 8) < 0) {
            perror("Listen");
            exit(EXIT_FAILURE);
        }
        std::cout << "HTTP server started at http://" << host_ << ":" << port_ << std::endl;

        while (1) {
            if ((new_socket = accept(server_fd, (struct sockaddr *)&address,
                                     (socklen_t*)&addrlen)) < 0) {
                perror("Accept");
                continue;
            }
            std::thread(&HTTPServer::handle_client, this, new_socket).detach();
        }
    }

private:
    std::string host_;
    int port_;
    ROS2Publisher& ros2pub_;

    void handle_client(int client_sock) {
        char buffer[8192] = {0};
        int valread = read(client_sock, buffer, sizeof(buffer)-1);
        if (valread <= 0) {
            close(client_sock);
            return;
        }
        buffer[valread] = 0;
        std::string req(buffer);

        std::string method, path, http_version;
        std::unordered_map<std::string, std::string> headers;
        std::string body;

        parse_http_request(req, method, path, http_version, headers, body);

        json respj;
        int status = 200;
        std::string statusmsg = "OK";

        // Only five API endpoints
        if (method == "POST") {
            if (path == "/move/forward") {
                handle_move(body, 1.0, 0.0, 0.0, respj); // default linear.x=1.0
            } else if (path == "/move/backward") {
                handle_move(body, -1.0, 0.0, 0.0, respj);
            } else if (path == "/turn/left") {
                handle_move(body, 0.0, 0.0, 1.0, respj); // default angular.z=1.0
            } else if (path == "/turn/right") {
                handle_move(body, 0.0, 0.0, -1.0, respj);
            } else if (path == "/stop") {
                ros2pub_.publishTwist(0.0, 0.0, 0.0);
                respj["result"] = "stopped";
            } else {
                status = 404;
                statusmsg = "Not Found";
                respj["error"] = "Unknown endpoint";
            }
        } else {
            status = 405;
            statusmsg = "Method Not Allowed";
            respj["error"] = "Unsupported HTTP method";
        }

        std::string respbody = respj.dump();
        std::ostringstream resp;
        resp << "HTTP/1.1 " << status << " " << statusmsg << "\r\n"
             << "Content-Type: application/json\r\n"
             << "Content-Length: " << respbody.size() << "\r\n"
             << "Connection: close\r\n\r\n"
             << respbody;
        send(client_sock, resp.str().c_str(), resp.str().size(), 0);
        close(client_sock);
    }

    static void parse_http_request(const std::string& req, std::string& method, std::string& path, std::string& version,
                                   std::unordered_map<std::string, std::string>& headers, std::string& body) {
        std::istringstream ss(req);
        std::string line;
        std::getline(ss, line);
        std::istringstream reqline(line);
        reqline >> method >> path >> version;
        std::transform(method.begin(), method.end(), method.begin(), ::toupper);

        while (std::getline(ss, line) && line != "\r") {
            size_t delim = line.find(":");
            if (delim != std::string::npos) {
                std::string key = line.substr(0, delim);
                std::string val = line.substr(delim+1);
                while (!val.empty() && (val[0]==' ' || val[0]=='\t')) val.erase(0,1);
                // Remove \r
                if (!val.empty() && val.back() == '\r') val.pop_back();
                headers[key] = val;
            }
        }
        // Body
        std::ostringstream bodyss;
        while (std::getline(ss, line)) {
            bodyss << line << "\n";
        }
        body = bodyss.str();
        if (!body.empty() && body.back() == '\n') body.pop_back();
    }

    void handle_move(const std::string& body, double default_lin_x, double default_lin_y, double default_ang_z, json& respj) {
        try {
            double lin_x = default_lin_x;
            double lin_y = default_lin_y;
            double ang_z = default_ang_z;
            if (!body.empty()) {
                auto jobj = json::parse(body);
                if (jobj.contains("linear_x")) lin_x = jobj["linear_x"];
                if (jobj.contains("linear_y")) lin_y = jobj["linear_y"];
                if (jobj.contains("angular_z")) ang_z = jobj["angular_z"];
                // For /turn/left and /turn/right: allow "angle" param (degrees/radians)
                if (jobj.contains("angle")) {
                    // Optionally, can add duration-based rotation in a real driver
                    // For now, just set angular.z = angle (rad/s) for one-shot message
                    ang_z = jobj["angle"];
                }
                if (jobj.contains("speed")) {
                    // Override lin_x or ang_z depending on the endpoint
                    if (default_lin_x != 0.0)
                        lin_x = jobj["speed"];
                    else if (default_ang_z != 0.0)
                        ang_z = jobj["speed"];
                }
            }
            ros2pub_.publishTwist(lin_x, lin_y, ang_z);
            respj["result"] = "sent";
            respj["linear_x"] = lin_x;
            respj["linear_y"] = lin_y;
            respj["angular_z"] = ang_z;
        } catch (const std::exception& e) {
            respj["error"] = std::string("Invalid JSON: ") + e.what();
        }
    }
};

// --------- Main -----------
int main() {
    ROS2Publisher ros2pub;
    HTTPServer server(HTTP_HOST, HTTP_PORT, ros2pub);
    server.run();
    return 0;
}
```
