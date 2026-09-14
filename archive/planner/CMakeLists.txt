cmake_minimum_required(VERSION 3.10)
project(RRT_Planner)

find_package(SDL2 REQUIRED)

# --- CHANGE IS HERE ---
# You must include ALL .cpp files: main.cpp, RRT.cpp, AND PRM.cpp
add_executable(rrt_planner main.cpp RRT.cpp PRM.cpp)
# ----------------------

if (WIN32)
    target_link_libraries(rrt_planner PRIVATE SDL2::SDL2 SDL2::SDL2main)
else()
    target_include_directories(rrt_planner PRIVATE ${SDL2_INCLUDE_DIRS})
    target_link_libraries(rrt_planner PRIVATE ${SDL2_LIBRARIES})
endif()
