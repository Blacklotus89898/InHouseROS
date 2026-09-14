# ihrs::transport - pluggable transports over a common PacketSender interface
#
# Always built: TCP / UDP (POSIX sockets, header PacketSender interface)
# Optional:     ZeroMQ (IHIRS_WITH_ZMQ), MQTT (IHIRS_WITH_MQTT), WebRTC (IHIRS_WITH_WEBRTC)

set(IHRS_TRANSPORT_SOURCES
    src/tcp_sender.cpp
    src/tcp_receiver.cpp
    src/udp_sender.cpp
    src/udp_receiver.cpp
)

add_library(ihrs_transport ${IHRS_TRANSPORT_SOURCES})
add_library(ihrs::transport ALIAS ihrs_transport)

target_include_directories(ihrs_transport PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(ihrs_transport PUBLIC Threads::Threads ihrs_warnings)

if(IHIRS_WITH_ZMQ)
  find_path(CPPZMQ_INCLUDE_DIR zmq.hpp)
  find_library(ZMQ_LIBRARY NAMES zmq)
  if(NOT (CPPZMQ_INCLUDE_DIR AND ZMQ_LIBRARY))
    message(WARNING "IHIRS_WITH_ZMQ is ON but cppzmq/libzmq were not found; ZeroMQ demo apps disabled")
  endif()
endif()
