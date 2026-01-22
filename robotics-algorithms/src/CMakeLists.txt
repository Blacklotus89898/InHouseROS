# Define the library source files
add_library(robotics_core
    common/math_utils.cpp
    control/pid.cpp
    simulators/viz/graph_visualizer.cpp
    planning/graph_search/astar.cpp  # <--- NEW
    localization/kalman_filter_1d.cpp # <--- NEW
    kinematics/two_link_arm.cpp  # <--- NEW
    # We will add pid.cpp, astar.cpp here later...
)

# Tell the compiler where to find the header files (.hpp)
# We also include Eigen here so the library can find it
target_include_directories(robotics_core PUBLIC 
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/third_party/Eigen
)


find_package(SDL2)
if(SDL2_FOUND)
    target_link_libraries(robotics_core PRIVATE ${SDL2_LIBRARIES})
    target_include_directories(robotics_core PRIVATE ${SDL2_INCLUDE_DIRS})
endif()
