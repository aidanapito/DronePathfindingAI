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
option(USE_ONNX "Use ONNX Runtime for inference" ON)
if(USE_ONNX)
    find_package(ONNXRuntime QUIET)
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

# Source files
file(GLOB_RECURSE SOURCES 
    "src/*.cpp"
    "src/sim/*.cpp"
    "src/agent/*.cpp"
    "src/bridge/*.cpp"
    "src/ui/*.cpp"
)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME} 
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
)

if(USE_ONNX AND ONNXRuntime_FOUND)
    target_link_libraries(${PROJECT_NAME} ONNXRuntime::ONNXRuntime)
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
