cmake_minimum_required(VERSION 3.16)
project(robotics_middleware LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_EXAMPLES "Build examples" ON)
option(BUILD_TOOLS "Build tools" ON)

add_library(robotics_core
  core/messaging/broker.cpp
  core/runtime/executor.cpp
)

target_include_directories(robotics_core
  PUBLIC ${PROJECT_SOURCE_DIR}/core
)

target_link_libraries(robotics_core pthread)

if (BUILD_EXAMPLES)
  add_executable(odometry_node examples/odometry_node.cpp)
  target_link_libraries(odometry_node robotics_core)

  add_executable(logger_node examples/logger_node.cpp)
  target_link_libraries(logger_node robotics_core)
endif()

if (BUILD_TOOLS)
  add_executable(topic_viewer core/tools/topic_viewer.cpp)
  target_link_libraries(topic_viewer robotics_core)
endif()

