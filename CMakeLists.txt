cmake_minimum_required(VERSION 3.17)
project(DronePathfindingAI VERSION 1.0.0 LANGUAGES CXX)

# Set C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Find required packages
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# ONNX Runtime (optional, for Track B)
option(USE_ONNX "Use ONNX Runtime for inference" OFF)
if(USE_ONNX)
    # Try to find ONNX Runtime - check multiple possible locations
    find_package(ONNXRuntime QUIET)
    if(NOT ONNXRuntime_FOUND)
        # Try to find it manually on macOS with Homebrew
        if(APPLE)
            find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
                PATHS /opt/homebrew/include /usr/local/include
                PATH_SUFFIXES onnxruntime
            )
            find_library(ONNXRUNTIME_LIBRARY
                NAMES onnxruntime
                PATHS /opt/homebrew/lib /usr/local/lib
            )
            if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIBRARY)
                set(ONNXRuntime_FOUND TRUE)
                set(ONNXRuntime_INCLUDE_DIRS ${ONNXRUNTIME_INCLUDE_DIR})
                set(ONNXRuntime_LIBRARIES ${ONNXRUNTIME_LIBRARY})
                message(STATUS "ONNX Runtime found manually: ${ONNXRUNTIME_LIBRARY}")
            endif()
        endif()
    endif()
    
    if(ONNXRuntime_FOUND)
        message(STATUS "ONNX Runtime found, enabling Track B")
        add_definitions(-DUSE_ONNX)
    else()
        message(WARNING "ONNX Runtime not found, Track B will be disabled")
        set(USE_ONNX OFF)
    endif()
endif()

# LibTorch (alternative to ONNX)
option(USE_LIBTORCH "Use LibTorch for inference" OFF)
if(USE_LIBTORCH AND NOT USE_ONNX)
    find_package(Torch QUIET)
    if(Torch_FOUND)
        message(STATUS "LibTorch found, enabling Track B")
        add_definitions(-DUSE_LIBTORCH)
    else()
        message(WARNING "LibTorch not found, Track B will be disabled")
        set(USE_LIBTORCH OFF)
    endif()
endif()

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

# Source files for main executable
set(MAIN_SOURCES
    "src/Simulator.cpp"
    "src/agent/Agent.cpp"
    "src/agent/FrameBuffer.cpp"
    "src/agent/QLearningAgent.cpp"
    "src/agent/VisionAgent.cpp"
    "src/bridge/Environment.cpp"
    "src/main.cpp"
    "src/sim/Drone.cpp"
    "src/sim/World.cpp"
    "src/ui/SimulatorUI.cpp"
)

# Debug: Print found source files
message(STATUS "Found main source files:")
foreach(SOURCE ${MAIN_SOURCES})
    message(STATUS "  ${SOURCE}")
endforeach()

# Create library target for core functionality
add_library(${PROJECT_NAME}_lib STATIC ${MAIN_SOURCES})

# Create main executable
add_executable(${PROJECT_NAME} src/main.cpp)

# Create test executable
add_executable(test_pathfinding src/test_pathfinding.cpp)

# Link libraries for library target
target_link_libraries(${PROJECT_NAME}_lib
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
)

# Link libraries for main executable
target_link_libraries(${PROJECT_NAME} 
    ${PROJECT_NAME}_lib
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
)

# Link libraries for test executable
target_link_libraries(test_pathfinding
    ${PROJECT_NAME}_lib
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
)

if(USE_ONNX AND ONNXRuntime_FOUND)
    if(TARGET ONNXRuntime::ONNXRuntime)
        target_link_libraries(${PROJECT_NAME} ONNXRuntime::ONNXRuntime)
    else()
        # Manual linking for manually found ONNX Runtime
        target_include_directories(${PROJECT_NAME} PRIVATE ${ONNXRuntime_INCLUDE_DIRS})
        target_link_libraries(${PROJECT_NAME} ${ONNXRuntime_LIBRARIES})
    endif()
endif()

if(USE_LIBTORCH AND Torch_FOUND)
    target_link_libraries(${PROJECT_NAME} ${TORCH_LIBRARIES})
    target_compile_definitions(${PROJECT_NAME} PRIVATE ${TORCH_CXX_FLAGS})
endif()

# Compiler flags
if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /W4)
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Install
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
