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

clean:
	rm -f bin/shm_demo bin/publisher bin/subscriber $(OBJS)

test:
	@echo "Testing in simulation"
	./bin/shm_demo
