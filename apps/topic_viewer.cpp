#include <iostream>
#include <string>
#include "ihrs/core/subscriber.hpp"
#include "ihrs/core/broker.hpp"

using namespace ihrs::core;

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

