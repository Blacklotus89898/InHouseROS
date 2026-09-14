cmake_minimum_required(VERSION 3.21)

project(InHouseROS
  VERSION 0.3.0
  DESCRIPTION "InHouseROS - lightweight robotics middleware and algorithms"
  LANGUAGES CXX
)

# ---------------------------------------------------------------------------
# Global toolchain settings
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------
option(IHIRS_BUILD_EXAMPLES "Build library examples" ON)
option(IHIRS_BUILD_APPS     "Build applications and tools" ON)
option(IHIRS_BUILD_TESTS    "Build unit tests" ON)
option(IHIRS_WITH_ZMQ       "Enable ZeroMQ transport" ON)
option(IHIRS_WITH_MQTT      "Enable MQTT transport" OFF)
option(IHIRS_WITH_WEBRTC    "Enable WebRTC transport" OFF)

# ---------------------------------------------------------------------------
# Shared compile warnings applied to all first-party targets
# ---------------------------------------------------------------------------
add_library(ihrs_warnings INTERFACE)
if(MSVC)
  target_compile_options(ihrs_warnings INTERFACE /W4 /permissive-)
else()
  target_compile_options(ihrs_warnings INTERFACE
    -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor -Wold-style-cast
    -Wcast-align -Woverloaded-virtual -Wconversion -Wsign-conversion
  )
endif()

# Relaxed profile for large pre-existing code bases being modernized gradually
# (keeps -Wall/-Wextra/-Wpedantic/-Wshadow, drops pedantic conversion checks)
add_library(ihrs_warnings_legacy INTERFACE)
if(MSVC)
  target_compile_options(ihrs_warnings_legacy INTERFACE /W3)
else()
  target_compile_options(ihrs_warnings_legacy INTERFACE
    -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor
  )
endif()

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
find_package(Threads REQUIRED)

# Must precede all add_subdirectory() calls so CTest wiring is generated
if(IHIRS_BUILD_TESTS)
  enable_testing()
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()

# ---------------------------------------------------------------------------
# Libraries
# ---------------------------------------------------------------------------
add_subdirectory(libs/core)
add_subdirectory(libs/ipc)
add_subdirectory(libs/transport)
add_subdirectory(libs/algo)

# ---------------------------------------------------------------------------
# Applications & tests
# ---------------------------------------------------------------------------
if(IHIRS_BUILD_APPS)
  add_subdirectory(apps)
endif()

if(IHIRS_BUILD_TESTS)
  add_subdirectory(tests)
endif()
