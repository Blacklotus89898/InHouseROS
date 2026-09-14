// Typed demo: stdin lines -> typed topic -> framed UDP datagrams.
// Replaces the old hand-wired SHM->UDP glue; the bridge library owns the
// forwarding now. Pair with udp_receiver (this repo) on the other end.
#include <ihrs/bridge/udp_bridge.hpp>
#include <ihrs/core/context.hpp>
#include <ihrs/core/executor.hpp>

#include <cstring>
#include <iostream>
#include <string>
#include <thread>

namespace {
struct Text64 {
    char text[64]{};
};
} // namespace

int main(int argc, char** argv) {
    const std::string ip = argc > 1 ? argv[1] : "127.0.0.1";
    const int port = argc > 2 ? std::stoi(argv[2]) : 9001;

    auto& ctx = ihrs::core::Context::global();
    auto node = ctx.create_node("chat_sender");

    auto pub = node.create_publisher<Text64>("/chat");
    ihrs::bridge::UdpTopicSink<Text64> sink(node, "/chat", ip, port);

    ihrs::core::Executor exec(ctx); // dispatches publish() to the sink
    std::thread spinner([&] { exec.spin(500.0); });

    std::cout << "chat_sender -> " << ip << ":" << port
              << " (type a line, Ctrl+D to exit)\n";
    std::string line;
    while (std::getline(std::cin, line)) {
        Text64 msg{};
        std::strncpy(msg.text, line.c_str(), sizeof(msg.text) - 1);
        pub->publish(msg);
    }

    exec.stop();
    spinner.join();
    return 0;
}
