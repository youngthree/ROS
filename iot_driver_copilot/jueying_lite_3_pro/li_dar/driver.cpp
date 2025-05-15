#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>
#include <map>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <atomic>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <json/json.h>

// --- ENV VARS ---
const char* ENV_DEVICE_IP         = "DEVICE_IP";
const char* ENV_UDP_PORT          = "DEVICE_UDP_PORT";
const char* ENV_HTTP_HOST         = "HTTP_SERVER_HOST";
const char* ENV_HTTP_PORT         = "HTTP_SERVER_PORT";
const char* ENV_UDP_TIMEOUT_MS    = "DEVICE_UDP_TIMEOUT_MS";

// --- CONSTANTS ---
const int    DEFAULT_HTTP_PORT    = 8080;
const char*  DEFAULT_HTTP_HOST    = "0.0.0.0";
const int    DEFAULT_UDP_PORT     = 9000;
const int    DEFAULT_UDP_TIMEOUT  = 2000; // ms
const size_t MAX_HTTP_REQ_SIZE    = 16384;
const size_t UDP_BUF_SIZE         = 4096;

// --- GLOBALS ---
std::mutex udp_mutex;
std::condition_variable udp_cv;
std::vector<uint8_t> udp_last_message;
std::atomic<bool> udp_data_ready(false);

// --- UTIL ---
std::string get_env(const char* key, const char* defval = nullptr) {
    const char* v = std::getenv(key);
    if (v && v[0]) return v;
    return defval ? defval : "";
}
int get_env_int(const char* key, int defval) {
    std::string s = get_env(key);
    if (s.empty()) return defval;
    return std::atoi(s.c_str());
}

// --- UDP RECEIVE THREAD ---
void udp_receiver_thread(std::string device_ip, int udp_port, int udp_timeout_ms) {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) return;

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(device_ip.c_str());
    addr.sin_port = htons(udp_port);

    if (bind(sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(sockfd);
        return;
    }

    struct timeval tv;
    tv.tv_sec = udp_timeout_ms / 1000;
    tv.tv_usec = (udp_timeout_ms % 1000) * 1000;
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    while (true) {
        uint8_t buf[UDP_BUF_SIZE];
        socklen_t addrlen = sizeof(addr);
        ssize_t n = recvfrom(sockfd, buf, sizeof(buf), 0, (struct sockaddr*)&addr, &addrlen);
        if (n > 0) {
            std::unique_lock<std::mutex> lock(udp_mutex);
            udp_last_message.assign(buf, buf+n);
            udp_data_ready = true;
            udp_cv.notify_all();
        }
    }
    close(sockfd);
}

// --- HTTP SERVER ---
class HttpServer {
    int server_fd;
    std::string host;
    int port;

public:
    HttpServer(const std::string& host_, int port_) : server_fd(-1), host(host_), port(port_) {}
    ~HttpServer() { if (server_fd >= 0) close(server_fd); }

    bool start() {
        server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd < 0) return false;

        int opt = 1;
        setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        struct sockaddr_in addr;
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = inet_addr(host.c_str());
        addr.sin_port = htons(port);

        if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            close(server_fd);
            return false;
        }
        if (listen(server_fd, 16) < 0) {
            close(server_fd);
            return false;
        }
        return true;
    }

    void run() {
        while (true) {
            int client_fd = accept(server_fd, nullptr, nullptr);
            if (client_fd < 0) continue;
            std::thread(&HttpServer::handle_client, this, client_fd).detach();
        }
    }

private:
    void handle_client(int client_fd) {
        char buf[MAX_HTTP_REQ_SIZE];
        ssize_t n = recv(client_fd, buf, sizeof(buf)-1, 0);
        if (n <= 0) { close(client_fd); return; }
        buf[n] = 0;
        std::string req(buf);
        std::string method, path, version;
        std::map<std::string, std::string> headers;
        std::string body;

        parse_http_request(req, method, path, version, headers, body);

        if (method == "POST" && path == "/simple") {
            handle_simple(client_fd, body);
        } else if (method == "POST" && path == "/complex") {
            handle_complex(client_fd, body);
        } else if (method == "POST" && path == "/move") {
            handle_move(client_fd, body);
        } else if (method == "GET" && path == "/udp-stream") {
            handle_udp_stream(client_fd);
        } else {
            send_http(client_fd, "404 Not Found", "text/plain", "Not Found", 404);
        }
        close(client_fd);
    }

    void parse_http_request(const std::string& req,
                            std::string& method, std::string& path, std::string& version,
                            std::map<std::string, std::string>& headers, std::string& body) {
        std::istringstream ss(req);
        std::string line;
        bool first = true;
        bool in_headers = true;
        while (std::getline(ss, line)) {
            if (line.empty() || line == "\r") {
                in_headers = false;
                continue;
            }
            if (first) {
                first = false;
                std::istringstream lss(line);
                lss >> method >> path >> version;
                continue;
            }
            if (in_headers) {
                auto pos = line.find(':');
                if (pos != std::string::npos) {
                    std::string key = line.substr(0, pos);
                    std::string val = line.substr(pos+1);
                    while (!val.empty() && (val[0] == ' ' || val[0] == '\t')) val.erase(0,1);
                    if (!val.empty() && val.back() == '\r') val.pop_back();
                    headers[key] = val;
                }
            } else {
                body += line;
            }
        }
    }

    void send_http(int client_fd, const std::string& status, const std::string& ctype, const std::string& body, int status_code=200) {
        std::ostringstream oss;
        oss << "HTTP/1.1 " << status << "\r\n";
        oss << "Content-Type: " << ctype << "\r\n";
        oss << "Content-Length: " << body.size() << "\r\n";
        oss << "Access-Control-Allow-Origin: *\r\n";
        oss << "\r\n";
        oss << body;
        std::string resp = oss.str();
        send(client_fd, resp.data(), resp.size(), 0);
    }

    void send_http_chunked_header(int client_fd, const std::string& status, const std::string& ctype) {
        std::ostringstream oss;
        oss << "HTTP/1.1 " << status << "\r\n";
        oss << "Content-Type: " << ctype << "\r\n";
        oss << "Transfer-Encoding: chunked\r\n";
        oss << "Access-Control-Allow-Origin: *\r\n";
        oss << "\r\n";
        std::string resp = oss.str();
        send(client_fd, resp.data(), resp.size(), 0);
    }

    void send_http_chunk(int client_fd, const void* data, size_t len) {
        char sizebuf[32];
        int sz = snprintf(sizebuf, sizeof(sizebuf), "%zx\r\n", len);
        send(client_fd, sizebuf, sz, 0);
        send(client_fd, data, len, 0);
        send(client_fd, "\r\n", 2, 0);
    }

    void send_http_chunked_end(int client_fd) {
        send(client_fd, "0\r\n\r\n", 5, 0);
    }

    // --- API HANDLERS ---
    void handle_simple(int client_fd, const std::string& body) {
        // Simulate dispatching to /simple_cmd
        bool ok = send_ros_command("simple_cmd", body);
        Json::Value resp;
        resp["result"] = ok ? "ok" : "failed";
        std::string out = resp.toStyledString();
        send_http(client_fd, "200 OK", "application/json", out);
    }

    void handle_complex(int client_fd, const std::string& body) {
        // Simulate dispatching to /complex_cmd
        bool ok = send_ros_command("complex_cmd", body);
        Json::Value resp;
        resp["result"] = ok ? "ok" : "failed";
        std::string out = resp.toStyledString();
        send_http(client_fd, "200 OK", "application/json", out);
    }

    void handle_move(int client_fd, const std::string& body) {
        // Simulate dispatching to /cmd_vel
        bool ok = send_ros_command("cmd_vel", body);
        Json::Value resp;
        resp["result"] = ok ? "ok" : "failed";
        std::string out = resp.toStyledString();
        send_http(client_fd, "200 OK", "application/json", out);
    }

    void handle_udp_stream(int client_fd) {
        send_http_chunked_header(client_fd, "200 OK", "application/octet-stream");
        while (true) {
            std::unique_lock<std::mutex> lock(udp_mutex);
            udp_cv.wait(lock, []{return udp_data_ready.load();});
            std::vector<uint8_t> data = udp_last_message;
            udp_data_ready = false;
            lock.unlock();
            if (!data.empty()) {
                send_http_chunk(client_fd, data.data(), data.size());
            }
        }
        send_http_chunked_end(client_fd);
    }

    // Simulated ROS command sender (stub for demo; replace with ROS integration)
    bool send_ros_command(const std::string& cmd, const std::string& payload) {
        // In a real driver, this would format and send a ROS message to the robot.
        // Here, we just simulate a success.
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        return true;
    }
};

int main() {
    std::string device_ip   = get_env(ENV_DEVICE_IP, "127.0.0.1");
    int udp_port            = get_env_int(ENV_UDP_PORT, DEFAULT_UDP_PORT);
    std::string http_host   = get_env(ENV_HTTP_HOST, DEFAULT_HTTP_HOST);
    int http_port           = get_env_int(ENV_HTTP_PORT, DEFAULT_HTTP_PORT);
    int udp_timeout_ms      = get_env_int(ENV_UDP_TIMEOUT_MS, DEFAULT_UDP_TIMEOUT);

    // Start UDP receiver thread
    std::thread(udp_receiver_thread, device_ip, udp_port, udp_timeout_ms).detach();

    // Start HTTP server
    HttpServer server(http_host, http_port);
    if (!server.start()) {
        std::cerr << "Failed to start HTTP server\n";
        return 1;
    }
    std::cout << "HTTP server running on " << http_host << ":" << http_port << std::endl;
    std::cout << "UDP stream available at /udp-stream\n";
    server.run();
    return 0;
}