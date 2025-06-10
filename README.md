# InHouseROS
Custom implementation of Robot Operating System using C++

## Motivation
- ROS build tool is too abstract
- Excessive amount of dependencies
- Performance limitation on current hardware
- For Learning :)

## Architecture
Client/Controller <-> Server/Robot
- Data stream of sensor feedback: gRPC
- Control input: gRPC
- Video stream: webrtc || raw tcp/udp packet

## Technology Stack
- ZeroMQ for msg passing 
- IPC, threading, POSIX shared memory + mutex
- Libdatachannel for webrtc like data channel
- C++, potential c/rust/zig addition

## Communication Paradigm
- Publisher - Subscriber
- OOP for the nodes

## Running the Program
```bash
# Makefile
make

# Simulating
./bin/shm_demo
# or
make test

# Running separately
make publisher
./bin/publisher

make subscriber
./bin/subscriber


```

## TODO
- [ ] Parametrize the publisher and subscriber
    - Rate
    - Callback function
