#include <mqtt/async_client.h>
#include <iostream>
#include <chrono>
#include <thread>

const std::string SERVER_ADDRESS("tcp://localhost:1883");
const std::string CLIENT_ID("cpp_sub");
const std::string TOPIC("test/topic");

class callback : public virtual mqtt::callback {
public:
    void message_arrived(mqtt::const_message_ptr msg) override {
        std::cout << "Received message on topic '" << msg->get_topic()
                  << "': " << msg->to_string() << std::endl;
    }

    void connection_lost(const std::string &cause) override {
        std::cerr << "Connection lost. Cause: " << cause << std::endl;
    }
};

int main() {
    mqtt::async_client client(SERVER_ADDRESS, CLIENT_ID);
    callback cb;
    client.set_callback(cb);

    mqtt::connect_options connOpts;
    try {
        std::cout << "Connecting to the MQTT broker..." << std::endl;
        client.connect(connOpts)->wait();
        std::cout << "Subscribing to topic: " << TOPIC << std::endl;
        client.subscribe(TOPIC, 1)->wait();

        std::cout << "Waiting for messages... Press Ctrl+C to exit.\n";
        while (true) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        client.disconnect()->wait();
    } catch (const mqtt::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
