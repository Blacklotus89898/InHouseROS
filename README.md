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

## Headers inside include
- PacketSender

## Classes (UpperCase) inside src
- pub-sub
    - SharedMemoryPublisher
    - SharedMemorySubscriber

- comms
    - UDPSender
    - UDPReceiver
    - tcp

- webrtc
    - caller
    - callee

## Function entrypoints (exapmples) inside programs
- main (testing script)
- publisher 
- subsriber

## Running the Program
```bash
# Makefile
make

# Simulating
./bin/shm_demo
# or
make test

# Running others
make publisher
./bin/publisher

make subscriber
./bin/subscriber

# Getting help
make help

```

## Expected output
```bash
$ make test
Testing in simulation
./bin/shm_demo
Shared memory initialized. Starting subscriber...
Shared memory initialized. Starting subscriber...
[Publisher] Sent: [Pub1] Count: 0
Sub1 received: [Pub1] Count: 0
[Publisher] Sent: [Pub2] Msg: 0
Sub2 received: [Pub2] Msg: 0
[Publisher] Sent: [Pub1] Count: 1
Sub1 received: [Pub1] Count: 1
[Publisher] Sent: [Pub2] Msg: 2
Sub2 received: [Pub2] Msg: 2
[Publisher] Sent: [Pub1] Count: 2
Sub2 received: [Pub1] Count: 2
Sub1 received: [Pub1] Count: 2
[Publisher] Sent: [Pub2] Msg: 4
[Publisher] Sent: [Pub1] Count: 3
Sub1 received: [Publisher] Sent: [Pub2] Msg: 6
[Pub1] Count: 3
Sub2 received: [Pub1] Count: 3
^C
Caught signal 2, cleaning up...
```

## TODO
- [x] Parametrize the publisher and subscriber
    - Rate -- to be [done]
    - msg type -- to be done [done]
    - Callback function [done]
- [ ] Raw implementation with threads only
- [ ] Wrapper to initiate all publishers and subscribers
- [ ] Make it a reusable library
- [ ] gRPC interface
- [ ] webRTC data channels
- [ ] TCP/UDP packet sender
- [ ] REST interface for log, kill signals
- [ ] Monitor/Observer tool for status
- [ ] Multi Sub/Pub node
- [ ] Pub + Sub node
- [ ] Tmux bash script for subsystem management