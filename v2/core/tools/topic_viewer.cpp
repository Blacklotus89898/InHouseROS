#include <iostream>
#include <string>
#include "messaging/subscriber.hpp"
#include "messaging/broker.hpp"

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: topic_viewer <topic>\n";
    return 1;
  }

  std::string topic = argv[1];

  Subscriber sub(topic, [&](MessagePtr) {
    std::cout << "[topic_viewer] message on '" << topic << "'\n";
  });

  while (true) {
    Broker::instance().spinOnce();
  }
}

