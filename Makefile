CXX = g++
CXXFLAGS = -std=c++23 -Wall -Wextra -pthread
INCLUDES = -Iinclude

SRCS = src/main.cpp src/SharedMemoryPublisher.cpp src/SharedMemorySubscriber.cpp
OBJS = $(SRCS:.cpp=.o)
TARGET = bin/shm_demo

all: $(TARGET)

# Create bin directory if it doesn't exist
$(shell mkdir -p bin)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@

src/main.o: src/main.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

src/SharedMemoryPublisher.o: src/SharedMemoryPublisher.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

src/SharedMemorySubscriber.o: src/SharedMemorySubscriber.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

publisher: src/publisher.cpp src/SharedMemoryPublisher.cpp
	@mkdir -p bin
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o bin/publisher

subscriber: src/subscriber.cpp src/SharedMemorySubscriber.cpp
	@mkdir -p bin
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o bin/subscriber

# UDP sender from subsriber
sender: src/sender.cpp src/SharedMemorySubscriber.cpp src/comms/UDPSender.cpp
	@mkdir -p bin
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o bin/sender

# UDP receiver -> publisher
receiver: src/receiver.cpp src/SharedMemoryPublisher.cpp src/comms/UDPReceiver.cpp
	@mkdir -p bin
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o bin/receiver -lpthread

# TCP implementation
tcpSender: src/tcpSender.cpp src/SharedMemorySubscriber.cpp src/comms/TCPSender.cpp
	@mkdir -p bin
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o bin/tcpSender

# UDP receiver -> publisher
tcpReceiver: src/tcpReceiver.cpp src/SharedMemoryPublisher.cpp src/comms/TCPReceiver.cpp
	@mkdir -p bin
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o bin/tcpReceiver -lpthread


clean:
	rm -f bin/shm_demo bin/publisher bin/subscriber bin/sender bin/receiver $(OBJS)

test:
	@echo "Testing in simulation"
	./bin/shm_demo
