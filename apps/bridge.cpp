#include <ihrs/bridge/shm_bridge.hpp>
#include <ihrs/bridge/udp_bridge.hpp>
#include <ihrs/core/context.hpp>
#include <ihrs/core/executor.hpp>

#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

// Payload types known to this bridge binary. Type identity is per-binary:
// both ends of a mapping must be built from the same registry.
struct Imu {
    double ax{0}, ay{0}, az{0};
    int seq{0};
};

struct Odometry {
    double x{0}, y{0}, theta{0};
    int seq{0};
};

struct Text64 {
    char text[64]{};
};

using AnyEndpoint = std::shared_ptr<void>; // type-erased, deleter retained

struct Mapping {
    std::string mode;   // shm-out | shm-in | udp-out | udp-in
    std::string topic;
    std::string a;      // shm: segment | udp-out: ip
    std::string b;      // udp: port
    std::string type;
};

using Factory = std::function<AnyEndpoint(ihrs::core::NodeHandle&, const Mapping&, int rate)>;

template <typename T>
AnyEndpoint make_endpoint(ihrs::core::NodeHandle& node, const Mapping& m, int rate) {
    if (m.mode == "shm-out") {
        return std::make_shared<ihrs::bridge::ShmTopicSink<T>>(node, m.topic, m.a, rate);
    }
    if (m.mode == "shm-in") {
        return std::make_shared<ihrs::bridge::ShmTopicSource<T>>(node, m.topic, m.a, rate);
    }
    if (m.mode == "udp-out") {
        return std::make_shared<ihrs::bridge::UdpTopicSink<T>>(node, m.topic, m.a,
                                                               std::stoi(m.b));
    }
    return std::make_shared<ihrs::bridge::UdpTopicSource<T>>(node, m.topic, std::stoi(m.a));
}

std::unordered_map<std::string, Factory> make_registry() {
    return {
        {"Imu", &make_endpoint<Imu>},
        {"Odometry", &make_endpoint<Odometry>},
        {"Text64", &make_endpoint<Text64>},
    };
}

// "topic:rest" -> topic + rest split on ':'
bool parse_mapping(const std::string& spec, const std::string& mode, Mapping& out) {
    std::vector<std::string> parts;
    std::stringstream ss(spec);
    std::string item;
    while (std::getline(ss, item, ':')) {
        parts.push_back(item);
    }
    out.mode = mode;
    if (mode.rfind("shm", 0) == 0 && parts.size() == 3) {
        out.topic = parts[0];
        out.a = parts[1];
        out.type = parts[2];
        return true;
    }
    if (mode == "udp-out" && parts.size() == 4) {
        out.topic = parts[0];
        out.a = parts[1];
        out.b = parts[2];
        out.type = parts[3];
        return true;
    }
    if (mode == "udp-in" && parts.size() == 3) {
        out.topic = parts[0];
        out.a = parts[1]; // port
        out.type = parts[2];
        return true;
    }
    return false;
}

void usage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0 << " [options]\n"
        << "\n"
        << "Bridges typed topics across transports (SHM segments, UDP).\n"
        << "\n"
        << "Options:\n"
        << "  --shm-out topic:segment:Type   topic -> SHM segment (creates it)\n"
        << "  --shm-in  topic:segment:Type   SHM segment -> topic (must exist)\n"
        << "  --udp-out topic:ip:port:Type   topic -> framed UDP datagrams\n"
        << "  --udp-in  topic:port:Type      framed UDP datagrams -> topic\n"
        << "  --rate hz                      SHM poll/publish rate (default 100)\n"
        << "  --spin hz                      executor rate (default 200)\n"
        << "\n"
        << "Types: Imu, Odometry, Text64 (see source; extend the registry as needed)\n"
        << "\nExample:\n"
        << "  " << argv0 << " --shm-out /imu:imu_shm:Imu --udp-in /cmd:9871:Text64\n";
}

} // namespace

int main(int argc, char** argv) {
    const auto registry = make_registry();
    std::vector<Mapping> mappings;
    int rate_hz = 100;
    int spin_hz = 200;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto next = [&]() -> std::string { return i + 1 < argc ? argv[++i] : ""; };

        if (arg == "--shm-out" || arg == "--shm-in" || arg == "--udp-out" ||
            arg == "--udp-in") {
            const std::string mode = arg.substr(2);
            const std::string spec = next();
            Mapping m;
            if (spec.empty() || !parse_mapping(spec, mode, m)) {
                std::cerr << "invalid mapping: " << spec << "\n";
                usage(argv[0]);
                return 1;
            }
            mappings.push_back(std::move(m));
        } else if (arg == "--rate") {
            rate_hz = std::stoi(next());
        } else if (arg == "--spin") {
            spin_hz = std::stoi(next());
        } else if (arg == "--help" || arg == "-h") {
            usage(argv[0]);
            return 0;
        } else {
            std::cerr << "unknown argument: " << arg << "\n";
            usage(argv[0]);
            return 1;
        }
    }

    if (mappings.empty()) {
        usage(argv[0]);
        return 1;
    }

    auto& ctx = ihrs::core::Context::global();
    auto node = ctx.create_node("bridge");
    std::vector<AnyEndpoint> endpoints;

    for (const Mapping& m : mappings) {
        const auto it = registry.find(m.type);
        if (it == registry.end()) {
            std::cerr << "unknown type: " << m.type << "\n";
            return 1;
        }
        try {
            endpoints.push_back(it->second(node, m, rate_hz));
            std::cout << "[bridge] " << m.mode << " topic='" << m.topic << "' type="
                      << m.type << (m.mode.rfind("udp", 0) == 0 ? "" : " segment=" + m.a)
                      << "\n";
        } catch (const std::exception& e) {
            std::cerr << "[bridge] failed to create " << m.mode << " for '" << m.topic
                      << "': " << e.what() << "\n";
            return 1;
        }
    }

    std::cout << "[bridge] running (Ctrl+C to exit)\n";
    ihrs::core::Executor exec(ctx);
    exec.spin(static_cast<double>(spin_hz)); // blocks until stop()/SIGINT kills us
    return 0;
}
