#include <ros/ros.h>
#include <ros/master.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Twist.h>
#include <yaml-cpp/yaml.h>

#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <httplib.h>
#include <fstream>
#include <cstdlib>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

// Kubernetes API interaction
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

// ---- Utils ----
using json = nlohmann::json;

// ---- Globals ----
std::string edgeDeviceName;
std::string edgeDeviceNamespace;
std::string k8sToken;
std::string k8sServer;
std::string deviceAddress;
std::string configPath = "/etc/edgedevice/config/instructions";
std::mutex statusMutex;
std::atomic<std::string> devicePhase{"Unknown"};

// ---- Helper Prototypes ----
std::string get_env(const std::string &var, const std::string &defaultVal = "");
void load_instruction_config(YAML::Node &yamlConf);
void update_k8s_phase(const std::string &phase);
bool get_edge_device_address(std::string &address);
bool check_device_connection();
void k8s_token_and_server();

// ---- ROS Data Caches ----
sensor_msgs::Imu latest_imu;
std::mutex imu_mutex;
nav_msgs::Odometry latest_odom;
std::mutex odom_mutex;
sensor_msgs::JointState latest_joints;
std::mutex joints_mutex;
std_msgs::String latest_us;
std::mutex us_mutex;
std_msgs::String latest_tracking;
std::mutex tracking_mutex;

// ---- Config ----
struct ApiInstructionConfig {
    std::map<std::string, std::string> protocolPropertyList;
};
std::map<std::string, ApiInstructionConfig> apiConfigs;

// ---- Utility Functions ----
std::string get_env(const std::string &var, const std::string &defaultVal) {
    auto v = getenv(var.c_str());
    return v ? std::string(v) : defaultVal;
}

// ---- Kubernetes API Functions ----
void k8s_token_and_server() {
    // Try to read the token from the default location
    std::ifstream tokenFile("/var/run/secrets/kubernetes.io/serviceaccount/token");
    if (tokenFile) {
        std::getline(tokenFile, k8sToken, '\0');
        tokenFile.close();
    }
    // Try to read the server address from env or default
    k8sServer = get_env("KUBERNETES_SERVICE_HOST", "") + std::string(":") + get_env("KUBERNETES_SERVICE_PORT", "");
    if (!k8sServer.empty()) {
        k8sServer = "https://" + k8sServer;
    }
}

bool get_edge_device_address(std::string &address) {
    // GET /apis/shifu.edgenesis.io/v1alpha1/namespaces/{ns}/edgedevices/{name}
    std::string url = k8sServer + "/apis/shifu.edgenesis.io/v1alpha1/namespaces/" + edgeDeviceNamespace + "/edgedevices/" + edgeDeviceName;

    CURL *curl = curl_easy_init();
    if (!curl) return false;
    struct curl_slist *headers = NULL;
    std::string bearer = "Authorization: Bearer " + k8sToken;
    headers = curl_slist_append(headers, bearer.c_str());
    headers = curl_slist_append(headers, "Accept: application/json");

    std::string respData;
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);

    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION,
        +[](char *ptr, size_t size, size_t nmemb, void *userdata) -> size_t {
            std::string &data = *static_cast<std::string *>(userdata);
            data.append(ptr, size * nmemb);
            return size * nmemb;
        });
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &respData);

    CURLcode res = curl_easy_perform(curl);
    bool ok = false;
    if (res == CURLE_OK) {
        try {
            auto jsonVal = json::parse(respData);
            if (jsonVal.contains("spec") && jsonVal["spec"].contains("address")) {
                address = jsonVal["spec"]["address"].get<std::string>();
                ok = true;
            }
        } catch (...) { ok = false; }
    }
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    return ok;
}

void update_k8s_phase(const std::string &phase) {
    // PATCH /apis/shifu.edgenesis.io/v1alpha1/namespaces/{ns}/edgedevices/{name}/status
    std::string url = k8sServer + "/apis/shifu.edgenesis.io/v1alpha1/namespaces/" + edgeDeviceNamespace + "/edgedevices/" + edgeDeviceName + "/status";
    CURL *curl = curl_easy_init();
    if (!curl) return;
    struct curl_slist *headers = NULL;
    std::string bearer = "Authorization: Bearer " + k8sToken;
    headers = curl_slist_append(headers, bearer.c_str());
    headers = curl_slist_append(headers, "Accept: application/json");
    headers = curl_slist_append(headers, "Content-Type: application/merge-patch+json");

    json status = {
        {"status", {
            {"edgeDevicePhase", phase}
        }}
    };
    std::string payload = status.dump();

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "PATCH");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload.c_str());
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);

    std::string respData;
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION,
        +[](char *ptr, size_t size, size_t nmemb, void *userdata) -> size_t {
            std::string &data = *static_cast<std::string *>(userdata);
            data.append(ptr, size * nmemb);
            return size * nmemb;
        });
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &respData);

    curl_easy_perform(curl);
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
}

// ---- Device Status Monitor ----
bool check_device_connection() {
    // Try to ping ROS master
    try {
        return ros::master::check();
    } catch (...) {
        return false;
    }
}

void device_status_monitor() {
    while (ros::ok()) {
        std::string phase;
        if (!check_device_connection()) {
            phase = "Pending";
        } else {
            // Try to get address
            std::string addr;
            if (!get_edge_device_address(addr)) {
                phase = "Unknown";
            } else {
                phase = "Running";
            }
        }
        if (devicePhase != phase) {
            devicePhase = phase;
            update_k8s_phase(phase);
        }
        std::this_thread::sleep_for(std::chrono::seconds(4));
    }
}

// ---- ConfigMap Parsing ----
void load_instruction_config(YAML::Node &yamlConf) {
    try {
        yamlConf = YAML::LoadFile(configPath);
        for (auto it = yamlConf.begin(); it != yamlConf.end(); ++it) {
            ApiInstructionConfig conf;
            if (it->second["protocolPropertyList"]) {
                for (auto pit = it->second["protocolPropertyList"].begin();
                        pit != it->second["protocolPropertyList"].end(); ++pit) {
                    conf.protocolPropertyList[pit->first.as<std::string>()] = pit->second.as<std::string>();
                }
            }
            apiConfigs[it->first.as<std::string>()] = conf;
        }
    } catch (...) {}
}

// ---- ROS Data Callbacks ----
void imu_callback(const sensor_msgs::Imu::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(imu_mutex);
    latest_imu = *msg;
}
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(odom_mutex);
    latest_odom = *msg;
}
void joints_callback(const sensor_msgs::JointState::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(joints_mutex);
    latest_joints = *msg;
}
void us_callback(const std_msgs::String::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(us_mutex);
    latest_us = *msg;
}
void tracking_callback(const std_msgs::String::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(tracking_mutex);
    latest_tracking = *msg;
}

// ---- Main API Handlers ----
void handle_get_imu(const httplib::Request &req, httplib::Response &res) {
    std::lock_guard<std::mutex> lock(imu_mutex);
    json j;
    j["header"] = { {"stamp", latest_imu.header.stamp.toSec()} };
    j["orientation"] = {
        {"x", latest_imu.orientation.x},
        {"y", latest_imu.orientation.y},
        {"z", latest_imu.orientation.z},
        {"w", latest_imu.orientation.w}
    };
    j["angular_velocity"] = {
        {"x", latest_imu.angular_velocity.x},
        {"y", latest_imu.angular_velocity.y},
        {"z", latest_imu.angular_velocity.z}
    };
    j["linear_acceleration"] = {
        {"x", latest_imu.linear_acceleration.x},
        {"y", latest_imu.linear_acceleration.y},
        {"z", latest_imu.linear_acceleration.z}
    };
    res.set_content(j.dump(), "application/json");
}

void handle_get_odom(const httplib::Request &req, httplib::Response &res) {
    std::lock_guard<std::mutex> lock(odom_mutex);
    json j;
    j["header"] = { {"stamp", latest_odom.header.stamp.toSec()} };
    j["pose"] = {
        {"position", {
            {"x", latest_odom.pose.pose.position.x},
            {"y", latest_odom.pose.pose.position.y},
            {"z", latest_odom.pose.pose.position.z}
        }},
        {"orientation", {
            {"x", latest_odom.pose.pose.orientation.x},
            {"y", latest_odom.pose.pose.orientation.y},
            {"z", latest_odom.pose.pose.orientation.z},
            {"w", latest_odom.pose.pose.orientation.w}
        }}
    };
    j["twist"] = {
        {"linear", {
            {"x", latest_odom.twist.twist.linear.x},
            {"y", latest_odom.twist.twist.linear.y},
            {"z", latest_odom.twist.twist.linear.z}
        }},
        {"angular", {
            {"x", latest_odom.twist.twist.angular.x},
            {"y", latest_odom.twist.twist.angular.y},
            {"z", latest_odom.twist.twist.angular.z}
        }}
    };
    res.set_content(j.dump(), "application/json");
}

void handle_get_joints(const httplib::Request &req, httplib::Response &res) {
    std::lock_guard<std::mutex> lock(joints_mutex);
    json j;
    j["header"] = { {"stamp", latest_joints.header.stamp.toSec()} };
    j["name"] = latest_joints.name;
    j["position"] = latest_joints.position;
    j["velocity"] = latest_joints.velocity;
    j["effort"] = latest_joints.effort;
    res.set_content(j.dump(), "application/json");
}

void handle_get_us(const httplib::Request &req, httplib::Response &res) {
    std::lock_guard<std::mutex> lock(us_mutex);
    res.set_content(latest_us.data, "application/json");
}

void handle_get_tracking(const httplib::Request &req, httplib::Response &res) {
    std::lock_guard<std::mutex> lock(tracking_mutex);
    res.set_content(latest_tracking.data, "application/json");
}

void handle_post_vel(const httplib::Request &req, httplib::Response &res, ros::Publisher &cmd_vel_pub, ros::Publisher &cmd_vel_corrected_pub) {
    try {
        json payload = json::parse(req.body);
        geometry_msgs::Twist twist_msg;
        if (payload.contains("linear")) {
            twist_msg.linear.x = payload["linear"].value("x", 0.0);
            twist_msg.linear.y = payload["linear"].value("y", 0.0);
            twist_msg.linear.z = payload["linear"].value("z", 0.0);
        }
        if (payload.contains("angular")) {
            twist_msg.angular.x = payload["angular"].value("x", 0.0);
            twist_msg.angular.y = payload["angular"].value("y", 0.0);
            twist_msg.angular.z = payload["angular"].value("z", 0.0);
        }
        if (payload.value("corrected", false)) {
            cmd_vel_corrected_pub.publish(twist_msg);
        } else {
            cmd_vel_pub.publish(twist_msg);
        }
        res.set_content(R"({"result":"ok"})", "application/json");
    } catch (...) {
        res.status = 400;
        res.set_content(R"({"error":"Bad payload"})", "application/json");
    }
}

void handle_post_simple(const httplib::Request &req, httplib::Response &res, ros::Publisher &simple_cmd_pub) {
    try {
        std_msgs::String msg;
        msg.data = req.body;
        simple_cmd_pub.publish(msg);
        res.set_content(R"({"result":"ok"})", "application/json");
    } catch (...) {
        res.status = 400;
        res.set_content(R"({"error":"Bad payload"})", "application/json");
    }
}

void handle_post_complex(const httplib::Request &req, httplib::Response &res, ros::Publisher &complex_cmd_pub) {
    try {
        std_msgs::String msg;
        msg.data = req.body;
        complex_cmd_pub.publish(msg);
        res.set_content(R"({"result":"ok"})", "application/json");
    } catch (...) {
        res.status = 400;
        res.set_content(R"({"error":"Bad payload"})", "application/json");
    }
}

void handle_post_nav(const httplib::Request &req, httplib::Response &res, ros::Publisher &nav_cmd_pub) {
    try {
        std_msgs::String msg;
        msg.data = req.body;
        nav_cmd_pub.publish(msg);
        res.set_content(R"({"result":"ok"})", "application/json");
    } catch (...) {
        res.status = 400;
        res.set_content(R"({"error":"Bad payload"})", "application/json");
    }
}

void handle_post_track(const httplib::Request &req, httplib::Response &res, ros::Publisher &track_cmd_pub) {
    try {
        std_msgs::String msg;
        msg.data = req.body;
        track_cmd_pub.publish(msg);
        res.set_content(R"({"result":"ok"})", "application/json");
    } catch (...) {
        res.status = 400;
        res.set_content(R"({"error":"Bad payload"})", "application/json");
    }
}

// ---- Main ----
int main(int argc, char **argv) {
    // --- ENV ---
    edgeDeviceName = get_env("EDGEDEVICE_NAME", "");
    edgeDeviceNamespace = get_env("EDGEDEVICE_NAMESPACE", "");
    std::string ros_master_uri = get_env("ROS_MASTER_URI", "http://localhost:11311");
    std::string server_host = get_env("SHIFU_HTTP_SERVER_HOST", "0.0.0.0");
    int server_port = std::stoi(get_env("SHIFU_HTTP_SERVER_PORT", "8080"));

    if (edgeDeviceName.empty() || edgeDeviceNamespace.empty()) {
        fprintf(stderr, "EDGEDEVICE_NAME and EDGEDEVICE_NAMESPACE env required\n");
        return 1;
    }

    // --- Kubernetes API setup ---
    k8s_token_and_server();

    // --- Load ConfigMap ---
    YAML::Node yamlConf;
    load_instruction_config(yamlConf);

    // --- ROS Node ---
    ros::init(argc, argv, "device_shifu_driver");
    ros::NodeHandle nh;

    // --- ROS Publishers ---
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Publisher cmd_vel_corrected_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_corrected", 10);
    ros::Publisher simple_cmd_pub = nh.advertise<std_msgs::String>("/simple_cmd", 10);
    ros::Publisher complex_cmd_pub = nh.advertise<std_msgs::String>("/complex_cmd", 10);
    ros::Publisher nav_cmd_pub = nh.advertise<std_msgs::String>("/nav_cmd", 10);
    ros::Publisher track_cmd_pub = nh.advertise<std_msgs::String>("/track_cmd", 10);

    // --- ROS Subscribers ---
    ros::Subscriber imu_sub = nh.subscribe("/imu/data", 10, imu_callback);
    ros::Subscriber odom_sub = nh.subscribe("/leg_odom", 10, odom_callback);
    ros::Subscriber joints_sub = nh.subscribe("/joint_states", 10, joints_callback);
    ros::Subscriber us_sub = nh.subscribe("/us_publisher/ultrasound_distance", 10, us_callback);
    ros::Subscriber tracking_sub = nh.subscribe("/tracking_data", 10, tracking_callback);

    // --- Device Status Monitor Thread ---
    std::thread statusThread(device_status_monitor);

    // --- HTTP Server ---
    httplib::Server svr;

    // Register endpoints
    svr.Post("/commands/vel", [&](const httplib::Request &req, httplib::Response &res) {
        handle_post_vel(req, res, cmd_vel_pub, cmd_vel_corrected_pub);
    });
    svr.Post("/commands/simple", [&](const httplib::Request &req, httplib::Response &res) {
        handle_post_simple(req, res, simple_cmd_pub);
    });
    svr.Post("/commands/complex", [&](const httplib::Request &req, httplib::Response &res) {
        handle_post_complex(req, res, complex_cmd_pub);
    });
    svr.Post("/commands/nav", [&](const httplib::Request &req, httplib::Response &res) {
        handle_post_nav(req, res, nav_cmd_pub);
    });
    svr.Post("/commands/track", [&](const httplib::Request &req, httplib::Response &res) {
        handle_post_track(req, res, track_cmd_pub);
    });

    svr.Get("/sensors/imu", handle_get_imu);
    svr.Get("/sensors/odom", handle_get_odom);
    svr.Get("/sensors/joints", handle_get_joints);
    svr.Get("/sensors/us", handle_get_us);
    svr.Get("/tracking", handle_get_tracking);

    // --- Main loop ---
    std::thread httpThread([&] {
        svr.listen(server_host.c_str(), server_port);
    });

    ros::spin();

    svr.stop();
    httpThread.join();
    statusThread.join();

    return 0;
}