#include <zmq.hpp>
#include <string>
#include <iostream>

int main() {
    zmq::context_t context(1);
    zmq::socket_t subscriber(context, zmq::socket_type::sub);

    subscriber.connect("tcp://localhost:5555");

    subscriber.set(zmq::sockopt::subscribe, "topic1");

    while (true) {
        zmq::message_t message;
        subscriber.recv(message, zmq::recv_flags::none);
        std::string data(static_cast<char*>(message.data()), message.size());
        std::cout << "Received: " << data << std::endl;
    }

    return 0;
}
