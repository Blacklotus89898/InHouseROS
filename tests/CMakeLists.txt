# Core middleware tests
add_executable(ihrs_core_tests
    core/test_broker.cpp
    core/test_executor.cpp
    core/test_context_timers.cpp
)
target_link_libraries(ihrs_core_tests PRIVATE ihrs::core GTest::gtest_main)
gtest_discover_tests(ihrs_core_tests TIMEOUT 60)

# Transport (TCP/UDP loopback) tests
add_executable(ihrs_transport_tests
    transport/test_sockets.cpp
    transport/test_framing.cpp
)
target_link_libraries(ihrs_transport_tests PRIVATE ihrs::transport GTest::gtest_main)
gtest_discover_tests(ihrs_transport_tests TIMEOUT 60)

# Shared-memory IPC tests
add_executable(ihrs_ipc_tests
    ipc/test_shm.cpp
)
target_link_libraries(ihrs_ipc_tests PRIVATE ihrs::ipc GTest::gtest_main)
gtest_discover_tests(ihrs_ipc_tests TIMEOUT 60)
