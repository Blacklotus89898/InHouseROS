# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++23 -Wall -Wextra -pthread
INCLUDES = -Iinclude

# Directories
SRC_DIR = src
OBJ_DIR = obj
BIN_DIR = bin
PROG_DIR = $(SRC_DIR)/programs
PUBSUB_DIR = $(SRC_DIR)/pubSub
COMMS_DIR = $(SRC_DIR)/comms

# Create necessary directories
$(shell mkdir -p $(OBJ_DIR) $(BIN_DIR))

# Main target
MAIN_SRCS = $(PROG_DIR)/main.cpp $(PUBSUB_DIR)/SharedMemoryPublisher.cpp $(PUBSUB_DIR)/SharedMemorySubscriber.cpp
MAIN_OBJS = $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(subst $(SRC_DIR)/,,$(MAIN_SRCS)))
TARGET = $(BIN_DIR)/shm_demo

# Other programs
PUBLISHER_SRCS = $(PROG_DIR)/publisher.cpp $(PUBSUB_DIR)/SharedMemoryPublisher.cpp
SUBSCRIBER_SRCS = $(PROG_DIR)/subscriber.cpp $(PUBSUB_DIR)/SharedMemorySubscriber.cpp
SENDER_SRCS = $(PROG_DIR)/sender.cpp $(PUBSUB_DIR)/SharedMemorySubscriber.cpp $(COMMS_DIR)/UDPSender.cpp
RECEIVER_SRCS = $(PROG_DIR)/receiver.cpp $(PUBSUB_DIR)/SharedMemoryPublisher.cpp $(COMMS_DIR)/UDPReceiver.cpp
TCPSENDER_SRCS = $(PROG_DIR)/tcpSender.cpp $(PUBSUB_DIR)/SharedMemorySubscriber.cpp $(COMMS_DIR)/TCPSender.cpp
TCPRECEIVER_SRCS = $(PROG_DIR)/tcpReceiver.cpp $(PUBSUB_DIR)/SharedMemoryPublisher.cpp $(COMMS_DIR)/TCPReceiver.cpp
GPUB_SRCS = $(PROG_DIR)/genericPub.cpp
GSUB_SRCS = $(PROG_DIR)/genericSub.cpp
PSUB_SRCS = $(PROG_DIR)/pubSub.cpp
DUAL_SRCS =  $(PROG_DIR)/dualPubSub.cpp

# Default target
all: $(TARGET)

# Pattern rule to compile src/**/*.cpp to obj/**/*.o
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Link main target
$(TARGET): $(MAIN_OBJS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@

# Function to build secondary binaries
define build_program
$$(BIN_DIR)/$1: $$(subst $$(SRC_DIR)/,$$(OBJ_DIR)/,$$(patsubst %.cpp,%.o,$$($2_SRCS)))
	@mkdir -p $$(dir $$@)
	$$(CXX) $$(CXXFLAGS) $$(INCLUDES) $$^ -o $$@ $3

.PHONY: $1
$1: $$(BIN_DIR)/$1
endef

# Object file mapping
define to_objs
$(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(1))
endef

# List of builds
$(eval PUBLISHER_OBJS := $(call to_objs,$(PUBLISHER_SRCS)))
$(eval SUBSCRIBER_OBJS := $(call to_objs,$(SUBSCRIBER_SRCS)))
$(eval SENDER_OBJS := $(call to_objs,$(SENDER_SRCS)))
$(eval RECEIVER_OBJS := $(call to_objs,$(RECEIVER_SRCS)))
$(eval TCPSENDER_OBJS := $(call to_objs,$(TCPSENDER_SRCS)))
$(eval TCPRECEIVER_OBJS := $(call to_objs,$(TCPRECEIVER_SRCS)))
$(eval GPUB_OBJS := $(call to_objs,$(GPUB_SRCS)))
$(eval GSUB_OBJS := $(call to_objs,$(GSUB_SRCS)))
$(eval PSUB_OBJS := $(call to_objs,$(PSUB_SRCS)))
$(eval DUAL_OBJS := $(call to_objs,$(DUAL_SRCS)))

# Build other executables
$(eval $(call build_program,publisher,PUBLISHER,))
$(eval $(call build_program,subscriber,SUBSCRIBER,))
$(eval $(call build_program,sender,SENDER,))
$(eval $(call build_program,receiver,RECEIVER,-lpthread))
$(eval $(call build_program,tcpSender,TCPSENDER,))
$(eval $(call build_program,tcpReceiver,TCPRECEIVER,-lpthread))
$(eval $(call build_program,gPub,GPUB,))
$(eval $(call build_program,gSub,GSUB,))
$(eval $(call build_program,pubSub,PSUB,))
$(eval $(call build_program,dual,DUAL,))

# Clean target
.PHONY: clean
clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR)/*

# Help message
.PHONY: help
help:
	@echo "Usage: make [target]"
	@echo ""
	@echo "Targets:"
	@echo "  all         Build the main shm_demo binary"
	@echo "  publisher   Build the publisher binary"
	@echo "  subscriber  Build the subscriber binary"
	@echo "  sender      Build the UDP sender"
	@echo "  receiver    Build the UDP receiver"
	@echo "  tcpSender   Build the TCP sender"
	@echo "  tcpReceiver Build the TCP receiver"
	@echo "  gPub        Build the generic publisher"
	@echo "  gSub        Build the generic subscriber"
	@echo "  pubSub      Build the generic pubSub"
	@echo "  dual        Build the multi pubSub"
	@echo "  clean       Remove all binaries and object files"
	@echo "  test        Run the shm_demo program"
	@echo "  help        Show this message"

# Test target
.PHONY: test
test: $(TARGET)
	@echo "Running shm_demo..."
	@./$(TARGET)
