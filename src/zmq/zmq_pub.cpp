#include <zmq.hpp>
#include <string>
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    zmq::context_t context(1);
    zmq::socket_t publisher(context, zmq::socket_type::pub);
    publisher.bind("tcp://*:5555");

    std::this_thread::sleep_for(std::chrono::seconds(1)); // Allow subscriber to connect

    for (int i = 0; i < 10; ++i) {
        std::string topic = "topic1";
        std::string msg = "Hello ZeroMQ " + std::to_string(i);

        zmq::message_t topic_msg(topic.begin(), topic.end());
        zmq::message_t data_msg(msg.begin(), msg.end());

        publisher.send(topic_msg, zmq::send_flags::sndmore); // send topic
        publisher.send(data_msg, zmq::send_flags::none);     // send message

        std::cout << "Sent: " << topic << " - " << msg << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return 0;
}
