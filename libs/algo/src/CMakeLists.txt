set(IHRS_ALGO_SOURCES
    common/math_utils.cpp
    control/pid.cpp
    control/pure_pursuit.cpp
    control/mpc.cpp
    decision/fsm/state_machine.cpp
    decision/mdp_solver.cpp
    kinematics/two_link_arm.cpp
    kinematics/arm_3d.cpp
    kinematics/n_link_arm.cpp
    kinematics/six_link_arm.cpp
    localization/kalman_filter_1d.cpp
    localization/particle_filter.cpp
    perception/occupancy_grid.cpp
    perception/voxel_grid.cpp
    planning/graph_search/astar.cpp
    planning/sampling_based/rrt.cpp
    planning/sampling_based/rrt_star.cpp
    planning/sampling_based/prm.cpp
    simulators/viz/graph_visualizer.cpp
)

add_library(ihrs_algo ${IHRS_ALGO_SOURCES})
add_library(ihrs::algo ALIAS ihrs_algo)

target_include_directories(ihrs_algo PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_include_directories(ihrs_algo SYSTEM PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/Eigen
)

target_link_libraries(ihrs_algo PUBLIC ihrs_warnings_legacy)

# SDL2 is only required by the visualizer back-end
find_package(SDL2 QUIET)
if(SDL2_FOUND)
    target_link_libraries(ihrs_algo PRIVATE ${SDL2_LIBRARIES})
    target_include_directories(ihrs_algo PRIVATE ${SDL2_INCLUDE_DIRS})
    target_compile_definitions(ihrs_algo PRIVATE IHRS_HAS_SDL2=1)
endif()
