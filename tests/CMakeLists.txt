# Core middleware tests
add_executable(ihrs_core_tests
    core/test_broker.cpp
    core/test_executor.cpp
)
target_link_libraries(ihrs_core_tests PRIVATE ihrs::core GTest::gtest_main)
gtest_discover_tests(ihrs_core_tests)

# Transport (TCP/UDP loopback) tests
add_executable(ihrs_transport_tests
    transport/test_sockets.cpp
)
target_link_libraries(ihrs_transport_tests PRIVATE ihrs::transport GTest::gtest_main)
gtest_discover_tests(ihrs_transport_tests)

# Shared-memory IPC tests
add_executable(ihrs_ipc_tests
    ipc/test_shm.cpp
)
target_link_libraries(ihrs_ipc_tests PRIVATE ihrs::ipc GTest::gtest_main)
gtest_discover_tests(ihrs_ipc_tests)
