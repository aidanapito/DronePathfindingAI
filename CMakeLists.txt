cmake_minimum_required(VERSION 3.10)
project(DroneSim3D)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Include directories
include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})

# Create the main executable
add_executable(drone_sim 
    src/main.cpp
    src/Drone.cpp
    src/World.cpp
    src/Camera.cpp
    src/InputHandler.cpp
)

# Link libraries
target_link_libraries(drone_sim ${OpenCV_LIBS})

# Set output directory
set_target_properties(drone_sim PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
