#include <iostream>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <vector>
#include <sstream>
#include <map>
#include <mutex>
#include <chrono>
#include <condition_variable>
#include <atomic>
#include <csignal>
#include <cerrno>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

// ROS dependencies
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

// RTSP dependencies
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#define HTTP_OK "HTTP/1.1 200 OK\r\n"
#define HTTP_JSON_HEADER "Content-Type: application/json\r\n"
#define HTTP_MJPG_HEADER "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
#define HTTP_CORS_HEADER "Access-Control-Allow-Origin: *\r\n"
#define HTTP_END "\r\n"

using namespace std;

// ======================== Env Config =========================
struct Config {
    string device_ip;
    int ros_master_port;
    string ros_master_host;
    string video_rtsp_url;
    string server_host;
    int server_port;
    int video_http_port;
    string ros_namespace;

    static Config load_from_env() {
        Config cfg;
        cfg.device_ip = getenv("DEVICE_IP") ? getenv("DEVICE_IP") : "127.0.0.1";
        cfg.ros_master_port = getenv("ROS_MASTER_PORT") ? atoi(getenv("ROS_MASTER_PORT")) : 11311;
        cfg.ros_master_host = getenv("ROS_MASTER_HOST") ? getenv("ROS_MASTER_HOST") : "localhost";
        cfg.server_host = getenv("SERVER_HOST") ? getenv("SERVER_HOST") : "0.0.0.0";
        cfg.server_port = getenv("SERVER_PORT") ? atoi(getenv("SERVER_PORT")) : 8080;
        cfg.video_http_port = getenv("VIDEO_HTTP_PORT") ? atoi(getenv("VIDEO_HTTP_PORT")) : 8081;
        cfg.ros_namespace = getenv("ROS_NAMESPACE") ? getenv("ROS_NAMESPACE") : "";
        cfg.video_rtsp_url = getenv("VIDEO_RTSP_URL") ? getenv("VIDEO_RTSP_URL") : "rtsp://127.0.0.1:554/stream";
        return cfg;
    }
};

// ======================== ROS Data Cache =========================

struct SensorCache {
    nav_msgs::Odometry::ConstPtr odom;
    sensor_msgs::Imu::ConstPtr imu;
    sensor_msgs::JointState::ConstPtr joint_states;
    std_msgs::Int32::ConstPtr handle_state;
    std_msgs::Float32::ConstPtr ultrasound_distance;
    std::mutex mtx;
} sensor_cache;

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> l(sensor_cache.mtx);
    sensor_cache.odom = msg;
}
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg) {
    std::lock_guard<std::mutex> l(sensor_cache.mtx);
    sensor_cache.imu = msg;
}
void joint_cb(const sensor_msgs::JointState::ConstPtr& msg) {
    std::lock_guard<std::mutex> l(sensor_cache.mtx);
    sensor_cache.joint_states = msg;
}
void handle_cb(const std_msgs::Int32::ConstPtr& msg) {
    std::lock_guard<std::mutex> l(sensor_cache.mtx);
    sensor_cache.handle_state = msg;
}
void us_cb(const std_msgs::Float32::ConstPtr& msg) {
    std::lock_guard<std::mutex> l(sensor_cache.mtx);
    sensor_cache.ultrasound_distance = msg;
}

// ======================== HTTP Server Core =========================

class HttpServer {
public:
    HttpServer(const string& host, int port)
        : m_host(host), m_port(port), m_sock(-1), m_should_stop(false) {}

    ~HttpServer() { stop(); }

    void add_route(const string& method, const string& path,
                   function<void(int, const string&, const map<string, string>&, const string&)> handler) {
        routes[method + " " + path] = handler;
    }

    void start() {
        struct sockaddr_in server_addr;
        m_sock = socket(AF_INET, SOCK_STREAM, 0);
        if (m_sock < 0) {
            cerr << "Socket creation failed\n";
            exit(1);
        }
        int opt = 1;
        setsockopt(m_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = inet_addr(m_host.c_str());
        server_addr.sin_port = htons(m_port);
        if (bind(m_sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            cerr << "Bind failed\n";
            exit(1);
        }
        listen(m_sock, 8);
        m_thread = std::thread([this]() { this->accept_loop(); });
    }

    void stop() {
        m_should_stop = true;
        if (m_sock >= 0) close(m_sock);
        if (m_thread.joinable()) m_thread.join();
    }

private:
    void accept_loop() {
        while (!m_should_stop) {
            struct sockaddr_in cli_addr;
            socklen_t clilen = sizeof(cli_addr);
            int newsock = accept(m_sock, (struct sockaddr*)&cli_addr, &clilen);
            if (newsock < 0) continue;
            std::thread(&HttpServer::handle_client, this, newsock).detach();
        }
    }
    string parse_path_query(const string& uri, map<string, string>& query) {
        size_t qpos = uri.find('?');
        if (qpos == string::npos) return uri;
        string path = uri.substr(0, qpos);
        string qstr = uri.substr(qpos+1);
        stringstream ss(qstr);
        string kv;
        while (getline(ss, kv, '&')) {
            size_t eq = kv.find('=');
            if (eq != string::npos) {
                query[kv.substr(0, eq)] = kv.substr(eq+1);
            }
        }
        return path;
    }
    void handle_client(int sock) {
        char buf[4096];
        int n = recv(sock, buf, sizeof(buf)-1, 0);
        if (n <= 0) { close(sock); return; }
        buf[n] = 0;
        string req(buf);
        stringstream ss(req);
        string method, uri, proto;
        ss >> method >> uri >> proto;
        map<string, string> query;
        string path = parse_path_query(uri, query);
        string body;
        size_t header_end = req.find("\r\n\r\n");
        if (header_end != string::npos && header_end+4 < req.size()) {
            body = req.substr(header_end+4);
        }
        auto it = routes.find(method + " " + path);
        if (it != routes.end()) {
            it->second(sock, path, query, body);
        } else {
            // 404
            string resp = "HTTP/1.1 404 Not Found\r\n" HTTP_CORS_HEADER HTTP_END;
            send(sock, resp.c_str(), resp.length(), 0);
        }
        close(sock);
    }
    string m_host;
    int m_port;
    int m_sock;
    thread m_thread;
    atomic<bool> m_should_stop;
    map<string, function<void(int, const string&, const map<string, string>&, const string&)>> routes;
};

// ======================== /sensors Implementation =========================

string json_escape(const string& s) {
    ostringstream o;
    for (auto c : s) {
        if (c == '"' || c == '\\') o << '\\' << c;
        else if (c == '\n') o << "\\n";
        else o << c;
    }
    return o.str();
}

string sensors_json() {
    std::lock_guard<std::mutex> l(sensor_cache.mtx);
    ostringstream oss;
    oss << "{";
    bool first = true;
    if (sensor_cache.odom) {
        if (!first) oss << ",";
        first = false;
        oss << "\"leg_odom\":{"
            << "\"x\":" << sensor_cache.odom->pose.pose.position.x
            << ",\"y\":" << sensor_cache.odom->pose.pose.position.y
            << ",\"theta\":" << sensor_cache.odom->pose.pose.orientation.z
            << ",\"vx\":" << sensor_cache.odom->twist.twist.linear.x
            << ",\"vy\":" << sensor_cache.odom->twist.twist.linear.y
            << ",\"vtheta\":" << sensor_cache.odom->twist.twist.angular.z
            << "}";
    }
    if (sensor_cache.imu) {
        if (!first) oss << ",";
        first = false;
        oss << "\"imu/data\":{"
            << "\"ax\":" << sensor_cache.imu->linear_acceleration.x
            << ",\"ay\":" << sensor_cache.imu->linear_acceleration.y
            << ",\"az\":" << sensor_cache.imu->linear_acceleration.z
            << ",\"gx\":" << sensor_cache.imu->angular_velocity.x
            << ",\"gy\":" << sensor_cache.imu->angular_velocity.y
            << ",\"gz\":" << sensor_cache.imu->angular_velocity.z
            << "}";
    }
    if (sensor_cache.joint_states) {
        if (!first) oss << ",";
        first = false;
        oss << "\"joint_states\":{";
        oss << "\"names\":[";
        for (size_t i=0; i<sensor_cache.joint_states->name.size(); ++i) {
            if (i) oss << ",";
            oss << "\"" << json_escape(sensor_cache.joint_states->name[i]) << "\"";
        }
        oss << "],\"positions\":[";
        for (size_t i=0; i<sensor_cache.joint_states->position.size(); ++i) {
            if (i) oss << ",";
            oss << sensor_cache.joint_states->position[i];
        }
        oss << "]";
        oss << "}";
    }
    if (sensor_cache.handle_state) {
        if (!first) oss << ",";
        first = false;
        oss << "\"handle_state\":" << sensor_cache.handle_state->data;
    }
    if (sensor_cache.ultrasound_distance) {
        if (!first) oss << ",";
        first = false;
        oss << "\"us_publisher/ultrasound_distance\":" << sensor_cache.ultrasound_distance->data;
    }
    oss << "}";
    return oss.str();
}

// ======================== /move Implementation =========================

struct MovePublisher {
    ros::Publisher pub;
    std::mutex mtx;
    MovePublisher() {}
    void init(ros::NodeHandle& nh, const string& ns) {
        pub = nh.advertise<geometry_msgs::Twist>(ns + "/cmd_vel", 1);
    }
    void send_cmd(float vx, float vy, float vtheta) {
        geometry_msgs::Twist msg;
        msg.linear.x = vx;
        msg.linear.y = vy;
        msg.angular.z = vtheta;
        std::lock_guard<std::mutex> l(mtx);
        pub.publish(msg);
    }
} move_publisher;

// ======================== /video Implementation =========================

class VideoStreamer {
public:
    VideoStreamer(const string& rtsp_url) : m_rtsp_url(rtsp_url), m_stop(false) {}

    void start(int port) {
        m_thread = std::thread([this, port]() { this->server(port); });
    }
    void stop() {
        m_stop = true;
        if (m_thread.joinable()) m_thread.join();
    }
private:
    void server(int port) {
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) { cerr << "Video socket error\n"; return; }
        struct sockaddr_in addr;
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port);
        int opt = 1;
        setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            cerr << "Video bind failed\n";
            return;
        }
        listen(sock, 8);
        while (!m_stop) {
            struct sockaddr_in cli_addr;
            socklen_t clilen = sizeof(cli_addr);
            int cli_sock = accept(sock, (struct sockaddr*)&cli_addr, &clilen);
            if (cli_sock < 0) { continue; }
            std::thread(&VideoStreamer::handle_client, this, cli_sock).detach();
        }
        close(sock);
    }
    void handle_client(int sock) {
        string header = string(HTTP_OK) + HTTP_MJPG_HEADER + HTTP_CORS_HEADER + HTTP_END;
        send(sock, header.c_str(), header.length(), 0);
        cv::VideoCapture cap(m_rtsp_url, cv::CAP_FFMPEG);
        if (!cap.isOpened()) {
            close(sock);
            return;
        }
        cv::Mat frame;
        vector<uchar> jpg;
        while (cap.read(frame) && !m_stop) {
            jpg.clear();
            vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
            cv::imencode(".jpg", frame, jpg, params);
            ostringstream oss;
            oss << "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " << jpg.size() << "\r\n\r\n";
            send(sock, oss.str().c_str(), oss.str().length(), 0);
            send(sock, reinterpret_cast<char*>(jpg.data()), jpg.size(), 0);
            send(sock, "\r\n", 2, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        close(sock);
    }
    string m_rtsp_url;
    thread m_thread;
    atomic<bool> m_stop;
};

// ======================== Main =========================

int main(int argc, char** argv) {
    Config cfg = Config::load_from_env();

    // Set ROS master and namespace from env
    setenv("ROS_MASTER_URI", ("http://" + cfg.ros_master_host + ":" + std::to_string(cfg.ros_master_port)).c_str(), 1);
    if (!cfg.ros_namespace.empty())
        setenv("ROS_NAMESPACE", cfg.ros_namespace.c_str(), 1);

    ros::init(argc, argv, "lite3pro_http_driver");
    ros::NodeHandle nh;

    // Subscribe to sensors
    ros::Subscriber odom_sub = nh.subscribe(cfg.ros_namespace + "/leg_odom", 1, odom_cb);
    ros::Subscriber imu_sub = nh.subscribe(cfg.ros_namespace + "/imu/data", 1, imu_cb);
    ros::Subscriber joint_sub = nh.subscribe(cfg.ros_namespace + "/joint_states", 1, joint_cb);
    ros::Subscriber handle_sub = nh.subscribe(cfg.ros_namespace + "/handle_state", 1, handle_cb);
    ros::Subscriber us_sub = nh.subscribe(cfg.ros_namespace + "/us_publisher/ultrasound_distance", 1, us_cb);

    // Init movement publisher
    move_publisher.init(nh, cfg.ros_namespace);

    // Start HTTP server for /sensors and /move and /video redirect
    HttpServer http_srv(cfg.server_host, cfg.server_port);

    http_srv.add_route("GET", "/sensors",
        [](int sock, const string&, const map<string, string>&, const string&) {
            string body = sensors_json();
            string resp = string(HTTP_OK) + HTTP_JSON_HEADER + HTTP_CORS_HEADER +
                "Content-Length: " + std::to_string(body.length()) + "\r\n\r\n" +
                body;
            send(sock, resp.c_str(), resp.length(), 0);
        }
    );

    http_srv.add_route("POST", "/move",
        [](int sock, const string&, const map<string, string>&, const string& body) {
            // Expect JSON: {"vx":..., "vy":..., "vtheta":...}
            float vx=0, vy=0, vtheta=0;
            size_t p1 = body.find("\"vx\"");
            size_t p2 = body.find("\"vy\"");
            size_t p3 = body.find("\"vtheta\"");
            if (p1!=string::npos) vx = atof(body.c_str() + body.find(":",p1)+1);
            if (p2!=string::npos) vy = atof(body.c_str() + body.find(":",p2)+1);
            if (p3!=string::npos) vtheta = atof(body.c_str() + body.find(":",p3)+1);
            move_publisher.send_cmd(vx, vy, vtheta);
            string resp = string(HTTP_OK) + HTTP_JSON_HEADER + HTTP_CORS_HEADER + HTTP_END + "{\"status\":\"ok\"}";
            send(sock, resp.c_str(), resp.length(), 0);
        }
    );

    http_srv.add_route("GET", "/video",
        [cfg](int sock, const string&, const map<string, string>&, const string&) {
            // Provide a HTML page with MJPEG stream or the MJPEG endpoint
            stringstream html;
            html << HTTP_OK << "Content-Type: text/html\r\n" << HTTP_CORS_HEADER << HTTP_END;
            html << "<html><body><h1>Video Stream</h1>";
            html << "<img src='http://" << cfg.server_host << ":" << cfg.video_http_port << "/video_stream' />";
            html << "</body></html>";
            string s = html.str();
            send(sock, s.c_str(), s.length(), 0);
        }
    );

    // Start MJPEG /video_stream server (on VIDEO_HTTP_PORT)
    VideoStreamer video_streamer(cfg.video_rtsp_url);
    video_streamer.start(cfg.video_http_port);

    http_srv.add_route("GET", "/video_stream",
        [](int sock, const string&, const map<string, string>&, const string&) {
            // This path is handled by VideoStreamer on video_http_port
            string resp = "HTTP/1.1 404 Not Found\r\n" HTTP_CORS_HEADER HTTP_END;
            send(sock, resp.c_str(), resp.length(), 0);
        }
    );

    http_srv.start();

    // ROS event loop (spin in another thread to avoid blocking main)
    std::thread ros_spin([]() { ros::spin(); });

    cout << "Lite3 Pro HTTP driver running at http://" << cfg.server_host << ":" << cfg.server_port << endl;
    cout << "Video stream at http://" << cfg.server_host << ":" << cfg.video_http_port << "/video_stream" << endl;

    // Wait for shutdown
    signal(SIGINT, [](int){ exit(0); });
    ros_spin.join();
    return 0;
}