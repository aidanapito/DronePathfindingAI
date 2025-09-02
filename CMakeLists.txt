cmake_minimum_required(VERSION 3.10)
project(DronePathfindingAI)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenGL
find_package(OpenGL REQUIRED)

# Find GLFW3
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW3 REQUIRED glfw3)

# Find GLM
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLM REQUIRED glm)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${OPENGL_INCLUDE_DIRS})
include_directories(${GLFW3_INCLUDE_DIRS})
include_directories(${GLM_INCLUDE_DIRS})

# Source files
set(SOURCES
    src/main.cpp
    src/Drone.cpp
    src/World.cpp
    src/Camera.cpp
    src/InputHandler.cpp
    src/Renderer.cpp
    src/AI/PathfindingAI.cpp
    src/AI/PathPlanner.cpp
    src/AI/ObstacleAvoidance.cpp
    src/PythonAIIntegration.cpp
)

# Create executable
add_executable(drone_sim ${SOURCES})

# Link libraries
target_link_libraries(drone_sim 
    ${OPENGL_LIBRARIES}
    /opt/homebrew/lib/libglfw.dylib
    ${GLM_LIBRARIES}
    "-framework Cocoa"
    "-framework IOKit"
    "-framework CoreVideo"
)

# Set C++ standard
target_compile_features(drone_sim PRIVATE cxx_std_17)

# Add compiler flags to silence OpenGL deprecation warnings
target_compile_definitions(drone_sim PRIVATE GL_SILENCE_DEPRECATION)
