#include <iostream>
#include <string>
#include <cstdlib>
#include <sstream>
#include <vector>
#include <thread>
#include <map>
#include <mutex>
#include <algorithm>
#include <cstring>
#include <netinet/in.h>
#include <unistd.h>

// ==========================
// Configuration via ENV VARS
// ==========================
std::string get_env(const char* key, const std::string& def = "") {
    const char* val = getenv(key);
    if (val == nullptr) return def;
    return std::string(val);
}

const int MAX_REQUEST_SIZE = 65536;
const int MAX_RESPONSE_SIZE = 1048576;

// ==========================
// HTTP UTILITIES
// ==========================
struct HttpRequest {
    std::string method;
    std::string path;
    std::map<std::string, std::string> headers;
    std::string body;
};

struct HttpResponse {
    int status;
    std::string status_text;
    std::map<std::string, std::string> headers;
    std::string body;
};

std::string to_lower(const std::string& s) {
    std::string out = s;
    std::transform(out.begin(), out.end(), out.begin(), ::tolower);
    return out;
}

HttpRequest parse_request(const std::string& req) {
    std::istringstream iss(req);
    std::string line;
    HttpRequest http_req;
    bool first_line = true;
    // Parse request line and headers
    while (std::getline(iss, line) && line != "\r") {
        if (line.back() == '\r') line.pop_back();
        if (first_line) {
            std::istringstream lss(line);
            lss >> http_req.method >> http_req.path;
            first_line = false;
        } else {
            if (line.empty()) break;
            auto colon = line.find(':');
            if (colon != std::string::npos)
                http_req.headers[to_lower(line.substr(0, colon))] = line.substr(colon+2);
        }
    }
    // Read body if present
    if (http_req.headers.count("content-length")) {
        int len = std::stoi(http_req.headers["content-length"]);
        std::string body(len, 0);
        iss.read(&body[0], len);
        http_req.body = body;
    }
    return http_req;
}

std::string build_response(const HttpResponse& res) {
    std::ostringstream oss;
    oss << "HTTP/1.1 " << res.status << " " << res.status_text << "\r\n";
    for (const auto& kv : res.headers)
        oss << kv.first << ": " << kv.second << "\r\n";
    oss << "Content-Length: " << res.body.size() << "\r\n";
    oss << "\r\n";
    oss << res.body;
    return oss.str();
}

// ==========================
// ROS/CAN MOCKUP API
// ==========================
// Replace these with real CAN/ROS API as needed.
std::mutex robot_mutex;

bool can_send_command(const std::string& cmd, const std::string& data, std::string& error) {
    std::lock_guard<std::mutex> lock(robot_mutex);
    // Mock action: always succeed, unless unknown.
    // Here, you would encode cmd/data into CAN frames and interact with device.
    if (cmd == "mapsave" || cmd == "launch" || cmd == "update_tf" || cmd == "navgoal") {
        return true;
    }
    error = "Unknown command";
    return false;
}

// ==========================
// HANDLER LOGIC
// ==========================
HttpResponse handle_mapsave(const HttpRequest& req) {
    std::string error;
    if (can_send_command("mapsave", req.body, error)) {
        return {200, "OK", {{"Content-Type", "application/json"}}, "{\"status\":\"ok\",\"msg\":\"Map saved\"}"};
    }
    return {500, "Internal Server Error", {{"Content-Type", "application/json"}}, "{\"status\":\"error\",\"msg\":\""+error+"\"}"};
}

HttpResponse handle_launch(const HttpRequest& req) {
    std::string error;
    if (can_send_command("launch", req.body, error)) {
        return {200, "OK", {{"Content-Type", "application/json"}}, "{\"status\":\"ok\",\"msg\":\"Node launched\"}"};
    }
    return {500, "Internal Server Error", {{"Content-Type", "application/json"}}, "{\"status\":\"error\",\"msg\":\""+error+"\"}"};
}

HttpResponse handle_update_tf(const HttpRequest& req) {
    std::string error;
    if (can_send_command("update_tf", req.body, error)) {
        return {200, "OK", {{"Content-Type", "application/json"}}, "{\"status\":\"ok\",\"msg\":\"Transform updated\"}"};
    }
    return {500, "Internal Server Error", {{"Content-Type", "application/json"}}, "{\"status\":\"error\",\"msg\":\""+error+"\"}"};
}

HttpResponse handle_navgoal(const HttpRequest& req) {
    std::string error;
    if (can_send_command("navgoal", req.body, error)) {
        return {200, "OK", {{"Content-Type", "application/json"}}, "{\"status\":\"ok\",\"msg\":\"Navigation goal sent\"}"};
    }
    return {500, "Internal Server Error", {{"Content-Type", "application/json"}}, "{\"status\":\"error\",\"msg\":\""+error+"\"}"};
}

// ==========================
// HTTP SERVER
// ==========================
void handle_client(int client_sock) {
    char buffer[MAX_REQUEST_SIZE];
    ssize_t bytes = recv(client_sock, buffer, sizeof(buffer)-1, 0);
    if (bytes <= 0) {
        close(client_sock);
        return;
    }
    buffer[bytes] = 0;
    std::string request_str(buffer);
    HttpRequest req = parse_request(request_str);

    HttpResponse res;
    if (req.method == "POST" && req.path == "/mapsave") {
        res = handle_mapsave(req);
    } else if (req.method == "POST" && req.path == "/launch") {
        res = handle_launch(req);
    } else if (req.method == "POST" && req.path == "/update_tf") {
        res = handle_update_tf(req);
    } else if (req.method == "POST" && req.path == "/navgoal") {
        res = handle_navgoal(req);
    } else {
        res = {404, "Not Found", {{"Content-Type", "application/json"}}, "{\"status\":\"error\",\"msg\":\"Unknown endpoint\"}"};
    }

    std::string response_str = build_response(res);
    send(client_sock, response_str.c_str(), response_str.size(), 0);
    close(client_sock);
}

void start_server(const std::string& host, int port) {
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        perror("socket");
        exit(1);
    }

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = host == "0.0.0.0" ? INADDR_ANY : inet_addr(host.c_str());
    address.sin_port = htons(port);

    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        perror("bind");
        exit(1);
    }

    if (listen(server_fd, 8) < 0) {
        perror("listen");
        exit(1);
    }

    std::cout << "HTTP server started at " << host << ":" << port << std::endl;

    while (true) {
        int client_sock = accept(server_fd, NULL, NULL);
        if (client_sock < 0) continue;
        std::thread(handle_client, client_sock).detach();
    }
}

// ==========================
// MAIN
// ==========================
int main() {
    std::string http_host = get_env("SERVER_HOST", "0.0.0.0");
    int http_port = std::stoi(get_env("SERVER_PORT", "8080"));
    // Device IP, CAN ID, etc. can be used as needed.
    // std::string device_ip = get_env("DEVICE_IP", "192.168.0.10");
    // int can_id = std::stoi(get_env("CAN_ID", "1"));

    start_server(http_host, http_port);
    return 0;
}