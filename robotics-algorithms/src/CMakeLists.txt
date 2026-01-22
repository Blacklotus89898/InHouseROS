# Define the main library
add_library(robotics_core
    common/math_utils.cpp 
    # Add future files here...
)

# Allow the library to see public headers
target_include_directories(robotics_core PUBLIC 
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/third_party/Eigen
)
