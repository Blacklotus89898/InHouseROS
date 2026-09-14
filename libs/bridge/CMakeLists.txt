# ihrs::bridge - composition layer bridging typed topics across transports
# Header-only; depends on the three messaging libraries it composes.

add_library(ihrs_bridge INTERFACE)
add_library(ihrs::bridge ALIAS ihrs_bridge)

target_include_directories(ihrs_bridge INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(ihrs_bridge INTERFACE ihrs::core ihrs::ipc ihrs::transport)
