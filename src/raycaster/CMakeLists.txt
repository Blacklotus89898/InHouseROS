cmake_minimum_required(VERSION 3.10)
project(Wolf3D_Raycaster)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find SDL2 Library
find_package(SDL2 REQUIRED)

# Create the executable
# Replace "raycaster.cpp" with your actual file name if different
add_executable(raycaster raycaster.cpp)
add_executable(raycaster2 raycaster_advanced.cpp)
add_executable(slam slam.cpp)
add_executable(ekf ekf.cpp)

# Link SDL2
if (WIN32)
    # Windows usually creates the targets SDL2::SDL2 and SDL2::SDL2main
    target_link_libraries(raycaster PRIVATE SDL2::SDL2 SDL2::SDL2main)
    target_link_libraries(raycaster2 PRIVATE SDL2::SDL2 SDL2::SDL2main)
    target_link_libraries(slam PRIVATE SDL2::SDL2 SDL2::SDL2main)
    target_link_libraries(ekf PRIVATE SDL2::SDL2 SDL2::SDL2main)
else()
    # Linux/macOS often uses the variable method
    target_include_directories(raycaster PRIVATE ${SDL2_INCLUDE_DIRS})
    target_link_libraries(raycaster PRIVATE ${SDL2_LIBRARIES})
    target_include_directories(raycaster2 PRIVATE ${SDL2_INCLUDE_DIRS})
    target_link_libraries(raycaster2 PRIVATE ${SDL2_LIBRARIES})
    target_include_directories(slam PRIVATE ${SDL2_INCLUDE_DIRS})
    target_link_libraries(slam PRIVATE ${SDL2_LIBRARIES})
    target_include_directories(ekf PRIVATE ${SDL2_INCLUDE_DIRS})
    target_link_libraries(ekf PRIVATE ${SDL2_LIBRARIES})
endif()
