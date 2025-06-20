CXX = g++
CXXFLAGS = -Wall -std=c++17
LDFLAGS = -lpaho-mqttpp3 -lpaho-mqtt3as

SRC = mqtt_pub.cpp mqtt_sub.cpp
OBJS = $(SRC:.cpp=)
TARGETS = mqtt_pub mqtt_sub

all: $(TARGETS)

mqtt_pub: mqtt_pub.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS)

mqtt_sub: mqtt_sub.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm -f $(TARGETS)
