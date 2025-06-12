#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <stdexcept>
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>
#include "mqtt/async_client.h"

class MQTTDeviceDriver {
public:
    MQTTDeviceDriver() {
        // Get MQTT broker address from environment variable
        const char* broker_env = std::getenv("MQTT_BROKER_ADDRESS");
        if (!broker_env) {
            throw std::runtime_error("MQTT_BROKER_ADDRESS environment variable is required");
        }
        broker_address_ = broker_env;

        // Optional: Client ID from env or default to device name
        const char* client_id_env = std::getenv("MQTT_CLIENT_ID");
        if (client_id_env) {
            client_id_ = client_id_env;
        } else {
            client_id_ = "WheeltecROS2V3.5";
        }

        // Optional: Username/password for broker authentication
        const char* user_env = std::getenv("MQTT_USERNAME");
        const char* pass_env = std::getenv("MQTT_PASSWORD");
        if (user_env && pass_env) {
            username_ = user_env;
            password_ = pass_env;
        }

        // MQTT QOS (could be overridable)
        const char* qos_env = std::getenv("MQTT_QOS");
        if (qos_env) {
            qos_ = std::atoi(qos_env);
        } else {
            qos_ = 1;
        }

        // Connect timeout
        const char* timeout_env = std::getenv("MQTT_CONNECT_TIMEOUT");
        if (timeout_env) {
            conn_timeout_ = std::atoi(timeout_env);
        }

        // Topics (fixed for this device)
        robotdig_command_topic_ = "/robotdig/command";
        robotdog_action_topic_ = "/robotdog/action";

        connect();
    }

    ~MQTTDeviceDriver() {
        try {
            if (client_) {
                client_->disconnect()->wait();
            }
        } catch (...) {}
    }

    // API: PUBLISH /robotdig/command
    // Users call this to send a JSON payload to /robotdig/command
    bool publish_robotdig_command(const std::string& json_payload) {
        return internal_publish(robotdig_command_topic_, json_payload, qos_);
    }

    // API: PUBLISH /robotdog/action
    // Users call this to send a JSON payload to /robotdog/action
    bool publish_robotdog_action(const std::string& json_payload) {
        return internal_publish(robotdog_action_topic_, json_payload, qos_);
    }

private:
    std::string broker_address_;
    std::string client_id_;
    std::string username_, password_;
    int qos_ = 1;
    int conn_timeout_ = 10; // seconds
    std::string robotdig_command_topic_;
    std::string robotdog_action_topic_;

    std::unique_ptr<mqtt::async_client> client_;
    mqtt::connect_options conn_opts_;
    std::mutex publish_mutex_;

    void connect() {
        client_ = std::make_unique<mqtt::async_client>(broker_address_, client_id_);
        conn_opts_ = mqtt::connect_options_builder()
            .clean_session()
            .connect_timeout(std::chrono::seconds(conn_timeout_))
            .finalize();
        if (!username_.empty())
            conn_opts_.set_user_name(username_);
        if (!password_.empty())
            conn_opts_.set_password(password_);

        mqtt::token_ptr conntok;
        try {
            conntok = client_->connect(conn_opts_);
            conntok->wait();
        } catch (const mqtt::exception& exc) {
            throw std::runtime_error(std::string("Error connecting to MQTT broker: ") + exc.what());
        }
    }

    bool internal_publish(const std::string& topic, const std::string& payload, int qos) {
        std::lock_guard<std::mutex> lock(publish_mutex_);
        if (!client_ || !client_->is_connected()) {
            try {
                connect();
            } catch (...) {
                return false;
            }
        }
        mqtt::message_ptr pubmsg = mqtt::make_message(topic, payload);
        pubmsg->set_qos(qos);
        try {
            client_->publish(pubmsg)->wait();
        } catch (const mqtt::exception& exc) {
            return false;
        }
        return true;
    }
};

// ---------------- Exported C++ APIs for User ----------------

// Publish JSON payload to /robotdig/command
bool publish_robotdig_command(const std::string& json_payload) {
    static std::once_flag flag;
    static std::unique_ptr<MQTTDeviceDriver> driver;
    std::call_once(flag, []() {
        driver = std::make_unique<MQTTDeviceDriver>();
    });
    return driver->publish_robotdig_command(json_payload);
}

// Publish JSON payload to /robotdog/action
bool publish_robotdog_action(const std::string& json_payload) {
    static std::once_flag flag;
    static std::unique_ptr<MQTTDeviceDriver> driver;
    std::call_once(flag, []() {
        driver = std::make_unique<MQTTDeviceDriver>();
    });
    return driver->publish_robotdog_action(json_payload);
}