#include <cstdlib>
#include <cstring>
#include <iostream>
#include <thread>
#include <vector>
#include <sstream>
#include <map>
#include <mutex>
#include <chrono>
#include <atomic>
#include <condition_variable>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>

// ROS headers
#include <ros/ros.h>
#include <ros/init.h>
#include <ros/spinner.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

// MJPEG Encoding
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

// Globals for sensor data
std::mutex data_mutex;
nav_msgs::Odometry latest_leg_odom;
sensor_msgs::Imu latest_imu;
sensor_msgs::JointState latest_joint_states;
std_msgs::Int32 latest_handle_state;
std_msgs::Float32 latest_ultrasound_distance;
std::atomic<bool> got_leg_odom(false), got_imu(false), got_joint(false), got_handle(false), got_us(false);

// For video streaming
std::mutex video_mutex;
cv::Mat latest_frame;
std::atomic<bool> got_frame(false);
std::vector<std::condition_variable*> video_clients_cv;
std::vector<bool*> video_clients_flags;

// Get environment variable or fallback
std::string get_env(const char* var, const char* defval = nullptr) {
    const char* v = getenv(var);
    if (!v && defval) return defval;
    if (!v) return "";
    return std::string(v);
}

// ROS subscriber callbacks
void leg_odom_cb(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex);
    latest_leg_odom = *msg;
    got_leg_odom = true;
}
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex);
    latest_imu = *msg;
    got_imu = true;
}
void joint_states_cb(const sensor_msgs::JointState::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex);
    latest_joint_states = *msg;
    got_joint = true;
}
void handle_state_cb(const std_msgs::Int32::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex);
    latest_handle_state = *msg;
    got_handle = true;
}
void us_distance_cb(const std_msgs::Float32::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex);
    latest_ultrasound_distance = *msg;
    got_us = true;
}
void image_cb(const sensor_msgs::Image::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(video_mutex);
    try {
        latest_frame = cv_bridge::toCvCopy(msg, "bgr8")->image.clone();
        got_frame = true;
        for (size_t i = 0; i < video_clients_cv.size(); ++i) {
            if (video_clients_flags[i]) *(video_clients_flags[i]) = true;
            if (video_clients_cv[i]) video_clients_cv[i]->notify_all();
        }
    } catch (...) {}
}

// Simple HTTP server
class HTTPServer {
    int server_fd;
    std::string host;
    int port;
    std::atomic<bool> running;
    std::thread listen_thread;
    ros::NodeHandle* nh;

public:
    HTTPServer(const std::string& h, int p, ros::NodeHandle* nh_) : host(h), port(p), nh(nh_) {}
    ~HTTPServer() { stop(); }

    void start() {
        running = true;
        listen_thread = std::thread(&HTTPServer::run, this);
    }

    void stop() {
        running = false;
        if (listen_thread.joinable()) listen_thread.join();
        if (server_fd > 0) close(server_fd);
    }

private:
    static void send_all(int sock, const char* data, size_t len) {
        size_t sent = 0;
        while (sent < len) {
            ssize_t n = send(sock, data + sent, len - sent, 0);
            if (n <= 0) break;
            sent += n;
        }
    }

    void run() {
        struct sockaddr_in addr;
        server_fd = socket(AF_INET, SOCK_STREAM, 0);
        int opt = 1;
        setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        addr.sin_addr.s_addr = inet_addr(host.c_str());
        if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            perror("bind failed");
            exit(1);
        }
        listen(server_fd, 5);
        while (running) {
            int client = accept(server_fd, NULL, NULL);
            if (client < 0) continue;
            std::thread(&HTTPServer::handle, this, client).detach();
        }
    }

    void handle(int client) {
        char buf[4096] = {0};
        ssize_t r = recv(client, buf, sizeof(buf)-1, 0);
        if (r <= 0) {
            close(client);
            return;
        }
        std::string req(buf);
        std::istringstream iss(req);
        std::string line;
        getline(iss, line);
        std::string method, path, version;
        std::istringstream lss(line);
        lss >> method >> path >> version;
        if (method == "GET" && path == "/sensors") {
            handle_sensors(client);
            close(client);
        } else if (method == "GET" && path == "/video") {
            handle_video(client);
            // Do NOT close(client) here; handled inside handle_video
        } else if (method == "POST" && path == "/move") {
            std::string content_length;
            while (getline(iss, line) && line != "\r") {
                if (line.find("Content-Length:") == 0)
                    content_length = line.substr(15);
            }
            int clen = content_length.empty() ? 0 : atoi(content_length.c_str());
            std::string body;
            if (clen > 0) {
                body.resize(clen);
                recv(client, &body[0], clen, 0);
            }
            handle_move(client, body);
            close(client);
        } else {
            std::string resp = "HTTP/1.1 404 Not Found\r\nContent-Type: text/plain\r\n\r\nNot Found";
            send_all(client, resp.c_str(), resp.size());
            close(client);
        }
    }

    void handle_sensors(int client) {
        std::lock_guard<std::mutex> lock(data_mutex);
        std::ostringstream oss;
        oss << "{";
        oss << "\"leg_odom\":{"
            << "\"position\":[" << latest_leg_odom.pose.pose.position.x << "," << latest_leg_odom.pose.pose.position.y << "," << latest_leg_odom.pose.pose.position.z << "],"
            << "\"orientation\":[" << latest_leg_odom.pose.pose.orientation.x << "," << latest_leg_odom.pose.pose.orientation.y << "," << latest_leg_odom.pose.pose.orientation.z << "," << latest_leg_odom.pose.pose.orientation.w << "],"
            << "\"linear_velocity\":[" << latest_leg_odom.twist.twist.linear.x << "," << latest_leg_odom.twist.twist.linear.y << "," << latest_leg_odom.twist.twist.linear.z << "],"
            << "\"angular_velocity\":[" << latest_leg_odom.twist.twist.angular.x << "," << latest_leg_odom.twist.twist.angular.y << "," << latest_leg_odom.twist.twist.angular.z << "]"
            << "},";
        oss << "\"imu\":{"
            << "\"orientation\":[" << latest_imu.orientation.x << "," << latest_imu.orientation.y << "," << latest_imu.orientation.z << "," << latest_imu.orientation.w << "],"
            << "\"angular_velocity\":[" << latest_imu.angular_velocity.x << "," << latest_imu.angular_velocity.y << "," << latest_imu.angular_velocity.z << "],"
            << "\"linear_acceleration\":[" << latest_imu.linear_acceleration.x << "," << latest_imu.linear_acceleration.y << "," << latest_imu.linear_acceleration.z << "]"
            << "},";
        oss << "\"joint_states\":{"
            << "\"name\":[";
        for (size_t i = 0; i < latest_joint_states.name.size(); ++i) {
            oss << "\"" << latest_joint_states.name[i] << "\"";
            if (i+1 < latest_joint_states.name.size()) oss << ",";
        }
        oss << "],\"position\":[";
        for (size_t i = 0; i < latest_joint_states.position.size(); ++i) {
            oss << latest_joint_states.position[i];
            if (i+1 < latest_joint_states.position.size()) oss << ",";
        }
        oss << "]";
        oss << "},";
        oss << "\"handle_state\":" << latest_handle_state.data << ",";
        oss << "\"ultrasound_distance\":" << latest_ultrasound_distance.data;
        oss << "}";
        std::string json = oss.str();
        std::ostringstream resp;
        resp << "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nAccess-Control-Allow-Origin: *\r\nContent-Length: " << json.size() << "\r\n\r\n" << json;
        send_all(client, resp.str().c_str(), resp.str().size());
    }

    void handle_video(int client) {
        std::string headers =
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
            "Cache-Control: no-cache\r\n"
            "Connection: close\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "\r\n";
        send_all(client, headers.c_str(), headers.size());

        std::condition_variable cv;
        bool new_img = false;
        {
            std::lock_guard<std::mutex> lock(video_mutex);
            video_clients_cv.push_back(&cv);
            video_clients_flags.push_back(&new_img);
        }
        try {
            while (running) {
                std::unique_lock<std::mutex> lk(video_mutex);
                cv.wait_for(lk, std::chrono::milliseconds(500), [&] { return new_img || !running; });
                if (!got_frame) continue;
                std::vector<uchar> buf;
                cv::imencode(".jpg", latest_frame, buf);
                std::ostringstream oss;
                oss << "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " << buf.size() << "\r\n\r\n";
                send_all(client, oss.str().c_str(), oss.str().size());
                send_all(client, (char*)buf.data(), buf.size());
                send_all(client, "\r\n", 2);
                new_img = false;
            }
        } catch (...) {}
        {
            std::lock_guard<std::mutex> lock(video_mutex);
            for (size_t i = 0; i < video_clients_cv.size(); ++i) {
                if (video_clients_cv[i] == &cv) {
                    video_clients_cv.erase(video_clients_cv.begin() + i);
                    video_clients_flags.erase(video_clients_flags.begin() + i);
                    break;
                }
            }
        }
        close(client);
    }

    void handle_move(int client, const std::string& body) {
        // Accept JSON body: {"linear":[x,y,z], "angular":[x,y,z]}
        geometry_msgs::Twist cmd;
        size_t l = body.find("\"linear\"");
        size_t a = body.find("\"angular\"");
        if (l != std::string::npos) {
            size_t s = body.find('[', l);
            size_t e = body.find(']', s);
            std::string v = body.substr(s+1, e-s-1);
            std::istringstream iss(v);
            std::string num;
            std::vector<double> vals;
            while (getline(iss, num, ',')) vals.push_back(atof(num.c_str()));
            if (vals.size() > 0) cmd.linear.x = vals[0];
            if (vals.size() > 1) cmd.linear.y = vals[1];
            if (vals.size() > 2) cmd.linear.z = vals[2];
        }
        if (a != std::string::npos) {
            size_t s = body.find('[', a);
            size_t e = body.find(']', s);
            std::string v = body.substr(s+1, e-s-1);
            std::istringstream iss(v);
            std::string num;
            std::vector<double> vals;
            while (getline(iss, num, ',')) vals.push_back(atof(num.c_str()));
            if (vals.size() > 0) cmd.angular.x = vals[0];
            if (vals.size() > 1) cmd.angular.y = vals[1];
            if (vals.size() > 2) cmd.angular.z = vals[2];
        }
        ros::Publisher* pub = nh ? new ros::Publisher(nh->advertise<geometry_msgs::Twist>("cmd_vel", 1)) : nullptr;
        if (pub) {
            pub->publish(cmd);
            delete pub;
            std::string resp = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n{\"ok\":true}";
            send_all(client, resp.c_str(), resp.size());
        } else {
            std::string resp = "HTTP/1.1 500 Internal Server Error\r\nContent-Type: application/json\r\n\r\n{\"ok\":false}";
            send_all(client, resp.c_str(), resp.size());
        }
    }
};

int main(int argc, char** argv) {
    // Read ENV
    std::string ros_master = get_env("ROS_MASTER_URI", "http://localhost:11311");
    std::string ros_ip = get_env("ROS_IP", "127.0.0.1");
    std::string device_ip = get_env("DEVICE_IP", "127.0.0.1");
    std::string server_host = get_env("SERVER_HOST", "0.0.0.0");
    int server_port = atoi(get_env("SERVER_PORT", "8000").c_str());
    std::string video_topic = get_env("VIDEO_TOPIC", "/camera/color/image_raw");

    setenv("ROS_MASTER_URI", ros_master.c_str(), 1);
    setenv("ROS_IP", ros_ip.c_str(), 1);

    // ROS Init
    ros::init(argc, argv, "lite3pro_http_driver");
    ros::NodeHandle nh;

    ros::Subscriber sub_leg_odom = nh.subscribe("leg_odom", 1, leg_odom_cb);
    ros::Subscriber sub_imu = nh.subscribe("imu/data", 1, imu_cb);
    ros::Subscriber sub_joint = nh.subscribe("joint_states", 1, joint_states_cb);
    ros::Subscriber sub_handle = nh.subscribe("handle_state", 1, handle_state_cb);
    ros::Subscriber sub_us = nh.subscribe("us_publisher/ultrasound_distance", 1, us_distance_cb);
    ros::Subscriber sub_img = nh.subscribe(video_topic, 1, image_cb);

    HTTPServer server(server_host, server_port, &nh);
    server.start();

    ros::AsyncSpinner spinner(2);
    spinner.start();

    while (ros::ok()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    server.stop();
    return 0;
}