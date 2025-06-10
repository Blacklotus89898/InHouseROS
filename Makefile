# Compiler and flags
CXX := g++
CXXFLAGS := -Wall -Wextra -pthread -Iinclude

# Directories
SRC_DIR := src
BUILD_DIR := build
BIN_DIR := bin

# Source files
PUBLISHER_SRC := $(SRC_DIR)/publisher.cpp
SUBSCRIBER_SRC := $(SRC_DIR)/subscriber.cpp
MAIN_SRC := $(SRC_DIR)/main.cpp

# Targets
PUBLISHER_BIN := $(BIN_DIR)/publisher
SUBSCRIBER_BIN := $(BIN_DIR)/subscriber
MAIN_BIN := $(BIN_DIR)/main

# Default target
all: dirs $(PUBLISHER_BIN) $(SUBSCRIBER_BIN) $(MAIN_BIN)

# Create directories if not exist
dirs:
	mkdir -p $(BIN_DIR) $(BUILD_DIR)

# Compile publisher
$(PUBLISHER_BIN): $(PUBLISHER_SRC)
	$(CXX) $(CXXFLAGS) $< -o $@

# Compile subscriber
$(SUBSCRIBER_BIN): $(SUBSCRIBER_SRC)
	$(CXX) $(CXXFLAGS) $< -o $@

# Compile main
$(MAIN_BIN): $(MAIN_SRC)
	$(CXX) $(CXXFLAGS) $< -o $@

# Clean build and binaries
clean:
	rm -rf $(BIN_DIR) $(BUILD_DIR)

.PHONY: all clean dirs
