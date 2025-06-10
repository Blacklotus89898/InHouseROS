CXX = g++
CXXFLAGS = -pthread
LDLIBS = -ldatachannel -lpthread

all: caller callee

caller: caller.cpp
	$(CXX) $(CXXFLAGS) -o caller caller.cpp $(LDLIBS)

callee: callee.cpp
	$(CXX) $(CXXFLAGS) -o callee callee.cpp $(LDLIBS)

clean:
	rm -f caller callee
