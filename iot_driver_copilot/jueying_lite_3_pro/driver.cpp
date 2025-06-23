#include <iostream>
#include <cstdlib>
#include <cstring>
#include <string>
#include <stdexcept>
#include <mqtt/async_client.h>

// Helper to fetch environment variable or throw if missing
std::string get_env(const char* var) {
    const char* val = std::getenv(var);
    if (!val) {
        throw std::runtime_error(std::string("Missing required environment variable: ") + var);
    }
    return std::string(val);
}

class JueyingLite3ProMQTTDriver {
public:
    JueyingLite3ProMQTTDriver()
        : broker_address(get_env("MQTT_BROKER_ADDRESS")),
          client_id("JueyingLite3Pro_" + std::to_string(std::rand())),
          mqtt_client(broker_address, client_id)
    {
        mqtt::connect_options connOpts;
        connOpts.set_keep_alive_interval(20);
        connOpts.set_clean_session(true);
        mqtt_client.connect(connOpts)->wait();
    }

    ~JueyingLite3ProMQTTDriver() {
        try {
            mqtt_client.disconnect()->wait();
        } catch (...) {}
    }

    // User API: Publish to /robotdog/command
    bool publish_robotdog_command(const std::string& json_payload) {
        return publish_topic("/robotdog/command", json_payload, 1);
    }

    // User API: Publish to /robgtdog/action
    bool publish_robgtdog_action(const std::string& json_payload) {
        return publish_topic("/robgtdog/action", json_payload, 1);
    }

private:
    const std::string broker_address;
    const std::string client_id;
    mqtt::async_client mqtt_client;

    // Internal publish method (used by the user exposed APIs)
    bool publish_topic(const std::string& topic, const std::string& payload, int qos) {
        try {
            mqtt::message_ptr pubmsg = mqtt::make_message(topic, payload);
            pubmsg->set_qos(qos);
            mqtt_client.publish(pubmsg)->wait_for(std::chrono::seconds(10));
            return true;
        } catch (const mqtt::exception& exc) {
            std::cerr << "MQTT Publish failed: " << exc.what() << std::endl;
            return false;
        }
    }
};

// Example usage (for demonstration; remove if not desired in the driver code):
// int main() {
//     try {
//         JueyingLite3ProMQTTDriver driver;
//         std::string json_cmd = "{\"move\": \"forward\", \"speed\": 0.5}";
//         driver.publish_robotdog_command(json_cmd);
//         std::string json_action = "{\"action\": \"sit\"}";
//         driver.publish_robgtdog_action(json_action);
//     } catch (const std::exception& e) {
//         std::cerr << "Error: " << e.what() << std::endl;
//         return 1;
//     }
//     return 0;
// }