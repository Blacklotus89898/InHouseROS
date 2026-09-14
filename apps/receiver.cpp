// Typed demo: framed UDP datagrams -> typed topic -> console.
// Pair with udp_receiver's sibling udp_sender (this repo).
#include <ihrs/bridge/udp_bridge.hpp>
#include <ihrs/core/context.hpp>
#include <ihrs/core/executor.hpp>

#include <cstring>
#include <iostream>
#include <string>

namespace {
struct Text64 {
    char text[64]{};
};
} // namespace

int main(int argc, char** argv) {
    const int port = argc > 1 ? std::stoi(argv[1]) : 9001;

    auto& ctx = ihrs::core::Context::global();
    auto node = ctx.create_node("chat_receiver");

    const auto sub = node.create_subscriber<Text64>(
        "/chat", [](const Text64& msg) { std::cout << "received: " << msg.text << "\n"; });
    ihrs::bridge::UdpTopicSource<Text64> source(node, "/chat", port);

    std::cout << "chat_receiver listening on udp/" << port << " (Ctrl+C to exit)\n";
    ihrs::core::Executor exec(ctx);
    exec.spin(500.0); // blocks until stop(); Ctrl+C terminates the process
    return 0;
}
