set(IHRS_ALGO_DEMOS
    hello_robotics
    control_demo
    pid_visualizer
    planning_demo
    astar_visualizer
    kalman_demo
    arm_kinematics_demo
    state_machine_demo
    rrt_demo
    rrt_star
    prm_demo
    voxel_demo
    mapping_demo
    navigation_demo
    pf_demo
    arm_3d_demo
    n_arm_demo
    six_dof_demo
    mpc_demo
    mdp_demo
)

foreach(demo IN LISTS IHRS_ALGO_DEMOS)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${demo}.cpp")
    add_executable(${demo} ${demo}.cpp)
    target_link_libraries(${demo} PRIVATE ihrs::algo)
  endif()
endforeach()
