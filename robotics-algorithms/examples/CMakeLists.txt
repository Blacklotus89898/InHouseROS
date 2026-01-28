# Only build examples if the source file exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/hello_robotics.cpp")
  add_executable(hello_robotics hello_robotics.cpp)

  # Link against your main library
  target_link_libraries(hello_robotics PRIVATE robotics_core)


  # PID Demo 
  add_executable(control_demo control_demo.cpp)
  target_link_libraries(control_demo PRIVATE robotics_core)

  # PID Viz Demo
  add_executable(pid_visualizer pid_visualizer.cpp)
  target_link_libraries(pid_visualizer PRIVATE robotics_core)

  # Add Planning Demo
  add_executable(planning_demo planning_demo.cpp)
  target_link_libraries(planning_demo PRIVATE robotics_core)

  add_executable(astar_visualizer astar_visualizer.cpp)
  target_link_libraries(astar_visualizer PRIVATE robotics_core)

  add_executable(kalman_demo kalman_demo.cpp)
  target_link_libraries(kalman_demo PRIVATE robotics_core)

  add_executable(arm_kinematics_demo arm_kinematics_demo.cpp)
  target_link_libraries(arm_kinematics_demo PRIVATE robotics_core)

  add_executable(state_machine_demo state_machine_demo.cpp)
  target_link_libraries(state_machine_demo PRIVATE robotics_core)

  add_executable(rrt_demo rrt_demo.cpp)
  target_link_libraries(rrt_demo PRIVATE robotics_core)

  add_executable(voxel_demo voxel_demo.cpp)
  target_link_libraries(voxel_demo PRIVATE robotics_core)

  add_executable(mapping_demo mapping_demo.cpp)
  target_link_libraries(mapping_demo PRIVATE robotics_core)

  add_executable(navigation_demo navigation_demo.cpp)
  target_link_libraries(navigation_demo PRIVATE robotics_core)

  add_executable(pf_demo pf_demo.cpp)
  target_link_libraries(pf_demo PRIVATE robotics_core)

  add_executable(arm_3d_demo arm_3d_demo.cpp)
  target_link_libraries(arm_3d_demo PRIVATE robotics_core)

  add_executable(n_arm_demo n_arm_demo.cpp)
  target_link_libraries(n_arm_demo PRIVATE robotics_core)

  add_executable(six_dof_demo six_dof_demo.cpp)
  target_link_libraries(six_dof_demo PRIVATE robotics_core)

  add_executable(prm_demo prm_demo.cpp)
  target_link_libraries(prm_demo PRIVATE robotics_core)

  add_executable(rrt_star rrt_star.cpp)
  target_link_libraries(rrt_star PRIVATE robotics_core)

  add_executable(mpc_demo mpc_demo.cpp)
  target_link_libraries(mpc_demo PRIVATE robotics_core)
endif()
