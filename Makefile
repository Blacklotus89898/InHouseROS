# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++23 -Wall -Wextra -pthread
INCLUDES = -Iinclude

# Directories
SRC_DIR = src
OBJ_DIR = obj
BIN_DIR = bin

# Main target sources and objects
SRCS = $(SRC_DIR)/main.cpp $(SRC_DIR)/SharedMemoryPublisher.cpp $(SRC_DIR)/SharedMemorySubscriber.cpp
OBJS = $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(SRCS))
TARGET = $(BIN_DIR)/shm_demo

# Other executables sources
PUBLISHER_SRCS = $(SRC_DIR)/publisher.cpp $(SRC_DIR)/SharedMemoryPublisher.cpp
SUBSCRIBER_SRCS = $(SRC_DIR)/subscriber.cpp $(SRC_DIR)/SharedMemorySubscriber.cpp
SENDER_SRCS = $(SRC_DIR)/sender.cpp $(SRC_DIR)/SharedMemorySubscriber.cpp $(SRC_DIR)/comms/UDPSender.cpp
RECEIVER_SRCS = $(SRC_DIR)/receiver.cpp $(SRC_DIR)/SharedMemoryPublisher.cpp $(SRC_DIR)/comms/UDPReceiver.cpp
TCPSENDER_SRCS = $(SRC_DIR)/tcpSender.cpp $(SRC_DIR)/SharedMemorySubscriber.cpp $(SRC_DIR)/comms/TCPSender.cpp
TCPRECEIVER_SRCS = $(SRC_DIR)/tcpReceiver.cpp $(SRC_DIR)/SharedMemoryPublisher.cpp $(SRC_DIR)/comms/TCPReceiver.cpp
GPUB_SRCS = $(SRC_DIR)/genericPub.cpp
GSUB_SRCS = $(SRC_DIR)/genericSub.cpp

# Help target
.PHONY: help
help:
	@echo "Usage: make [target]"
	@echo ""
	@echo "Targets:"
	@echo "  all         Build the main program ($(TARGET))"
	@echo "  publisher   Build the publisher executable"
	@echo "  subscriber  Build the subscriber executable"
	@echo "  sender      Build the UDP sender executable"
	@echo "  receiver    Build the UDP receiver executable"
	@echo "  tcpSender   Build the TCP sender executable"
	@echo "  tcpReceiver Build the TCP receiver executable"
	@echo "  gPub        Build the generic publisher executable"
	@echo "  gSub        Build the generic subscriber executable"
	@echo "  clean       Remove all build files and binaries"
	@echo "  test        Run the main program for testing"
	@echo "  help        Show this help message"

# Default target
all: $(TARGET)

# Create bin and obj directories if they don't exist
$(BIN_DIR):
	mkdir -p $(BIN_DIR)

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

# Pattern rule to compile any .cpp in src/ to .o in obj/
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Link main target
$(TARGET): $(OBJS) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@

# Utility function to build arbitrary executables with dependency lists
define build_executable
$1: $($(2)_SRCS) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $($(2)_SRCS) -o $(BIN_DIR)/$1
endef

# Build other executables
$(eval $(call build_executable,publisher,PUBLISHER))
$(eval $(call build_executable,subscriber,SUBSCRIBER))
$(eval $(call build_executable,sender,SENDER))
$(eval $(call build_executable,receiver,RECEIVER))
$(eval $(call build_executable,tcpSender,TCPSENDER))
$(eval $(call build_executable,tcpReceiver,TCPRECEIVER))
$(eval $(call build_executable,gPub,GPUB))
$(eval $(call build_executable,gSub,GSUB))

# Clean up generated files and binaries
.PHONY: clean
clean:
	rm -f $(OBJ_DIR)/*.o
	rm -f $(BIN_DIR)/shm_demo $(BIN_DIR)/publisher $(BIN_DIR)/subscriber \
		$(BIN_DIR)/sender $(BIN_DIR)/receiver $(BIN_DIR)/tcpSender $(BIN_DIR)/tcpReceiver \
		$(BIN_DIR)/gSub $(BIN_DIR)/gPub

# Test target to run main program
.PHONY: test
test: $(TARGET)
	@echo "Running simulation test..."
	@./$(TARGET)
