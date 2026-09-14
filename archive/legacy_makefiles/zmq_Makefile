CXX = g++
CXXFLAGS = -Wall -std=c++17
LDFLAGS = -lzmq

SRC = zmq_pub.cpp zmq_sub.cpp
TARGETS = zmq_pub zmq_sub

all: $(TARGETS)

zmq_pub: zmq_pub.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS)

zmq_sub: zmq_sub.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm -f $(TARGETS)
