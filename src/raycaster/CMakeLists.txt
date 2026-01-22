cmake_minimum_required(VERSION 3.10)
project(Wolf3D_Raycaster)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find SDL2 Library
find_package(SDL2 REQUIRED)

# Create the executable
# Replace "raycaster.cpp" with your actual file name if different
add_executable(raycaster raycaster.cpp)

# Link SDL2
if (WIN32)
    # Windows usually creates the targets SDL2::SDL2 and SDL2::SDL2main
    target_link_libraries(raycaster PRIVATE SDL2::SDL2 SDL2::SDL2main)
else()
    # Linux/macOS often uses the variable method
    target_include_directories(raycaster PRIVATE ${SDL2_INCLUDE_DIRS})
    target_link_libraries(raycaster PRIVATE ${SDL2_LIBRARIES})
endif()
