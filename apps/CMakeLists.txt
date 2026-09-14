# Demo applications and operator tools

# --- Core (v2 middleware) demos -------------------------------------------
add_executable(odometry_node odometry_node.cpp)
target_link_libraries(odometry_node PRIVATE ihrs::core)

add_executable(logger_node logger_node.cpp)
target_link_libraries(logger_node PRIVATE ihrs::core)

add_executable(topic_viewer topic_viewer.cpp)
target_link_libraries(topic_viewer PRIVATE ihrs::core)

# --- Shared-memory IPC demos (v1) -----------------------------------------
add_executable(shm_demo main.cpp)
target_link_libraries(shm_demo PRIVATE ihrs::ipc)

add_executable(shm_publisher publisher.cpp)
target_link_libraries(shm_publisher PRIVATE ihrs::ipc)

add_executable(shm_subscriber subscriber.cpp)
target_link_libraries(shm_subscriber PRIVATE ihrs::ipc)

add_executable(generic_shm_pub genericPub.cpp)
target_link_libraries(generic_shm_pub PRIVATE ihrs::ipc)

add_executable(generic_shm_sub genericSub.cpp)
target_link_libraries(generic_shm_sub PRIVATE ihrs::ipc)

add_executable(generic_shm_pubsub pubSub.cpp)
target_link_libraries(generic_shm_pubsub PRIVATE ihrs::ipc)

add_executable(dual_shm_pubsub dualPubSub.cpp)
target_link_libraries(dual_shm_pubsub PRIVATE ihrs::ipc)

# --- Network bridging demos (SHM <-> UDP / TCP) ---------------------------
add_executable(udp_sender sender.cpp)
target_link_libraries(udp_sender PRIVATE ihrs::bridge)

add_executable(udp_receiver receiver.cpp)
target_link_libraries(udp_receiver PRIVATE ihrs::bridge)

add_executable(bridge bridge.cpp)
target_link_libraries(bridge PRIVATE ihrs::bridge)

add_executable(tcp_sender tcpSender.cpp)
target_link_libraries(tcp_sender PRIVATE ihrs::ipc ihrs::transport)

add_executable(tcp_receiver tcpReceiver.cpp)
target_link_libraries(tcp_receiver PRIVATE ihrs::ipc ihrs::transport)

# --- Optional transports (only when dependencies exist) -------------------
if(IHIRS_WITH_ZMQ)
  find_path(CPPZMQ_INCLUDE_DIR zmq.hpp)
  find_library(ZMQ_LIBRARY NAMES zmq)
  if(CPPZMQ_INCLUDE_DIR AND ZMQ_LIBRARY)
    foreach(app zmq_demo_publisher zmq_demo_subscriber)
      add_executable(${app} ${app}.cpp)
      target_include_directories(${app} PRIVATE ${CPPZMQ_INCLUDE_DIR})
      target_link_libraries(${app} PRIVATE ${ZMQ_LIBRARY} Threads::Threads)
    endforeach()
  endif()
endif()

if(IHIRS_WITH_MQTT)
  find_path(PAHO_CPP_INCLUDE_DIR mqtt/async_client.h)
  find_library(PAHO_CPP_LIBRARY NAMES paho-cpp paho-mqttpp3)
  if(PAHO_CPP_INCLUDE_DIR AND PAHO_CPP_LIBRARY)
    foreach(app mqtt_demo_publisher mqtt_demo_subscriber)
      add_executable(${app} ${app}.cpp)
      target_include_directories(${app} PRIVATE ${PAHO_CPP_INCLUDE_DIR})
      target_link_libraries(${app} PRIVATE ${PAHO_CPP_LIBRARY} Threads::Threads)
    endforeach()
  else()
    message(WARNING "IHIRS_WITH_MQTT is ON but Paho C++ was not found")
  endif()
endif()

if(IHIRS_WITH_WEBRTC)
  find_package(datachannel QUIET)
  if(datachannel_FOUND)
    add_executable(webrtc_demo_caller webrtc_demo_caller.cpp)
    target_link_libraries(webrtc_demo_caller PRIVATE datachannel::datachannel)
    add_executable(webrtc_demo_callee webrtc_demo_callee.cpp)
    target_link_libraries(webrtc_demo_callee PRIVATE datachannel::datachannel)
    add_executable(webrtc_pub webRTCPub.cpp)
    target_link_libraries(webrtc_pub PRIVATE ihrs::ipc datachannel::datachannel)
    add_executable(webrtc_sub webRTCSub.cpp)
    target_link_libraries(webrtc_sub PRIVATE ihrs::ipc datachannel::datachannel)
  else()
    message(WARNING "IHIRS_WITH_WEBRTC is ON but libdatachannel was not found")
  endif()
endif()
