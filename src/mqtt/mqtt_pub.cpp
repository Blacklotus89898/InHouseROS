
#include <mqtt/async_client.h>

int main() {
    const std::string address = "tcp://localhost:1883";
    const std::string clientID = "cpp_pub";
    mqtt::async_client client(address, clientID);
    mqtt::connect_options connOpts;

    try {
        client.connect(connOpts)->wait();
        mqtt::message_ptr pubmsg = mqtt::make_message("test/topic", "Hello from C++!");
        pubmsg->set_qos(1);
        client.publish(pubmsg)->wait();
        client.disconnect()->wait();
        std::cout << "Message sent successfully." << std::endl;
    } catch (const mqtt::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
