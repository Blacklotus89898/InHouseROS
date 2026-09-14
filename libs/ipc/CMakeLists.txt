add_library(ihrs_ipc src/shm_publisher.cpp src/shm_subscriber.cpp)
add_library(ihrs::ipc ALIAS ihrs_ipc)

target_include_directories(ihrs_ipc PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(ihrs_ipc PUBLIC Threads::Threads ihrs_warnings)
