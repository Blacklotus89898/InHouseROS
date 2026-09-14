add_library(ihrs_core src/broker.cpp src/executor.cpp)
add_library(ihrs::core ALIAS ihrs_core)

target_include_directories(ihrs_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(ihrs_core PUBLIC Threads::Threads ihrs_warnings)
