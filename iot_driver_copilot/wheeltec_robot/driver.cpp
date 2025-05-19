#include <cstdlib>
#include <cstring>
#include <iostream>
#include <thread>
#include <chrono>
#include <sstream>
#include <csignal>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#ifdef _WIN32
#include <winsock2.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif

// Globals for clean shutdown
std::atomic<bool> running(true);

// Helper: Get env with default
std::string get_env(const char* var, const char* def) {
    const char* val = std::getenv(var);
    if (val) return std::string(val);
    return std::string(def);
}

// HTTP response helpers
std::string http_ok(const std::string& msg="OK") {
    std::ostringstream oss;
    oss << "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n{\"result\":\"" << msg << "\"}";
    return oss.str();
}
std::string http_notfound() {
    return "HTTP/1.1 404 Not Found\r\nContent-Type: application/json\r\n\r\n{\"error\":\"Not found\"}";
}
std::string http_error(const std::string& msg) {
    std::ostringstream oss;
    oss << "HTTP/1.1 400 Bad Request\r\nContent-Type: application/json\r\n\r\n{\"error\":\"" << msg << "\"}";
    return oss.str();
}

// Graceful exit
void signal_handler(int) {
    running = false;
}

// ROS2 Command Publisher
class WheeltecCommander : public rclcpp::Node {
public:
    WheeltecCommander(const std::string& name, const std::string& topic)
    : Node(name), cmd_topic_(topic) {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);
    }

    void move(double linear_x, double angular_z) {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = linear_x;
        msg.angular.z = angular_z;
        publisher_->publish(msg);
    }

    void stop() {
        move(0.0, 0.0);
    }

private:
    std::string cmd_topic_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

// Simple HTTP server (blocking, single-threaded, minimal for C++)
class SimpleHTTPServer {
public:
    SimpleHTTPServer(const std::string& host, int port, WheeltecCommander* commander)
    : host_(host), port_(port), commander_(commander) {}

    void run() {
#ifdef _WIN32
        WSADATA wsa;
        WSAStartup(MAKEWORD(2,2),&wsa);
#endif
        int server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd < 0) {
            std::cerr << "Socket creation failed\n";
            return;
        }
        int opt = 1;
        setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, (char*)&opt, sizeof(opt));
        sockaddr_in addr;
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port_);
        addr.sin_addr.s_addr = host_ == "0.0.0.0" ? INADDR_ANY : inet_addr(host_.c_str());
        if (bind(server_fd, (sockaddr*)&addr, sizeof(addr)) < 0) {
            std::cerr << "Bind failed\n";
#ifdef _WIN32
            closesocket(server_fd);
            WSACleanup();
#else
            close(server_fd);
#endif
            return;
        }
        if (listen(server_fd, 5) < 0) {
            std::cerr << "Listen failed\n";
#ifdef _WIN32
            closesocket(server_fd);
            WSACleanup();
#else
            close(server_fd);
#endif
            return;
        }
        while (running) {
            sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            int client_fd =
#ifdef _WIN32
                accept(server_fd, (sockaddr*)&client_addr, &client_len);
#else
                accept(server_fd, (sockaddr*)&client_addr, &client_len);
#endif
            if (client_fd < 0) {
                if (!running) break;
                continue;
            }
            char buffer[2048] = {0};
            int bytes_read =
#ifdef _WIN32
                recv(client_fd, buffer, sizeof(buffer)-1, 0);
#else
                read(client_fd, buffer, sizeof(buffer)-1);
#endif
            if (bytes_read > 0) {
                buffer[bytes_read] = '\0';
                std::string response = handle_http(std::string(buffer, bytes_read));
#ifdef _WIN32
                send(client_fd, response.c_str(), (int)response.size(), 0);
                closesocket(client_fd);
#else
                write(client_fd, response.c_str(), response.size());
                close(client_fd);
#endif
            } else {
#ifdef _WIN32
                closesocket(client_fd);
#else
                close(client_fd);
#endif
            }
        }
#ifdef _WIN32
        closesocket(server_fd);
        WSACleanup();
#else
        close(server_fd);
#endif
    }

private:
    std::string handle_http(const std::string& req) {
        // Parse HTTP method and path
        std::istringstream iss(req);
        std::string method, path;
        iss >> method >> path;
        if (method != "POST") return http_error("Only POST supported");

        // Routing
        if (path == "/move/forward") {
            commander_->move(0.3, 0.0); // Adjust speed as appropriate
            return http_ok("moving forward");
        }
        if (path == "/move/backward") {
            commander_->move(-0.3, 0.0);
            return http_ok("moving backward");
        }
        if (path == "/turn/left") {
            commander_->move(0.0, 0.7);
            return http_ok("turning left");
        }
        if (path == "/turn/right") {
            commander_->move(0.0, -0.7);
            return http_ok("turning right");
        }
        if (path == "/stop") {
            commander_->stop();
            return http_ok("stopped");
        }
        return http_notfound();
    }

    std::string host_;
    int port_;
    WheeltecCommander* commander_;
};

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::string ros_domain_id = get_env("ROS_DOMAIN_ID", "0");
    setenv("ROS_DOMAIN_ID", ros_domain_id.c_str(), 1);

    // HTTP server config
    std::string http_host = get_env("HTTP_SERVER_HOST", "0.0.0.0");
    int http_port = std::stoi(get_env("HTTP_SERVER_PORT", "8080"));

    // ROS2 config
    std::string ros_cmd_topic = get_env("ROS_CMD_VEL_TOPIC", "/cmd_vel");

    rclcpp::init(argc, argv);
    auto commander = std::make_shared<WheeltecCommander>("wheeltec_commander", ros_cmd_topic);

    // Spin ROS2 in background thread
    std::thread ros_spin([&]() {
        rclcpp::Rate rate(20);
        while (rclcpp::ok() && running) {
            rclcpp::spin_some(commander);
            rate.sleep();
        }
    });

    // HTTP server in this thread
    SimpleHTTPServer server(http_host, http_port, commander.get());
    server.run();

    running = false;
    rclcpp::shutdown();
    ros_spin.join();
    return 0;
}