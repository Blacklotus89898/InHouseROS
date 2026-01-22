# Only build examples if the source file exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/hello_robotics.cpp")
  add_executable(hello_robotics hello_robotics.cpp)

  # Link against your main library
  target_link_libraries(hello_robotics PRIVATE robotics_core)


  # New demo stuff
  add_executable(control_demo control_demo.cpp)
  target_link_libraries(control_demo PRIVATE robotics_core)

  add_executable(pid_visualizer pid_visualizer.cpp)
  target_link_libraries(pid_visualizer PRIVATE robotics_core)
endif()
