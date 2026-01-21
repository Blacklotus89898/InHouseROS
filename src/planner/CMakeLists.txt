cmake_minimum_required(VERSION 3.10)
project(RRT_Planner)

# 1. Find the SDL2 library installed on your system
find_package(SDL2 REQUIRED)

# 2. Define the executable (using your main.cpp file)
add_executable(rrt_planner main.cpp)

# 3. Link SDL2 libraries
if (WIN32)
    # Windows linking (assuming you are using a standard SDL2 distribution)
    target_link_libraries(rrt_planner PRIVATE SDL2::SDL2 SDL2::SDL2main)
else()
    # Linux/macOS linking
    target_include_directories(rrt_planner PRIVATE ${SDL2_INCLUDE_DIRS})
    target_link_libraries(rrt_planner PRIVATE ${SDL2_LIBRARIES})
endif()
