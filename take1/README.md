# Drone Pathfinding AI

A self-contained C++17 project implementing intelligent drone navigation using reinforcement learning, featuring both discrete Q-learning and vision-based deep learning approaches.

<!-- Repository language detection fix applied -->

## 🚁 Project Overview

This project implements a lightweight drone simulator with two learning tracks:

- **Track A (Q-Learning)**: Traditional reinforcement learning using discrete state spaces and Q-tables
- **Track B (Vision)**: Deep learning approach using ONNX Runtime for real-time vision processing

## 🎯 Current Status & Roadmap

### ✅ **COMPLETED (August 2024)**

#### **Core Infrastructure**
- [x] **Drone Simulation Engine** - Complete physics, collision detection, boundary enforcement
- [x] **World Generation** - Maze generation with obstacles and pathfinding support
- [x] **Agent Framework** - Base agent interface with Q-Learning implementation
- [x] **Environment Bridge** - RL environment wrapper with episode management
- [x] **Pathfinding Algorithms** - A* and Flood Fill (BFS) pathfinding
- [x] **Action Execution System** - Complete drone control with safety checks
- [x] **Stuck Detection & Recovery** - Advanced agent behavior with panic mode
- [x] **Testing Suite** - Comprehensive test coverage for all components
- [x] **Build System** - CMake configuration with proper dependency management
- [x] **Repository Cleanup** - Fixed GitHub language detection (was showing 96% Makefile, now properly shows C++)

#### **Q-Learning Implementation (Track A)**
- [x] **State Discretization** - Position, heading, and goal-based state representation
- [x] **Action Space** - Forward, turn left, turn right, idle actions
- [x] **Reward Shaping** - Goal reward, collision penalty, progress tracking
- [x] **Pathfinding Integration** - Hybrid navigation using optimal paths + Q-Learning
- [x] **Episode Management** - Complete episode lifecycle with reset and statistics
- [x] **Performance Optimization** - Efficient Q-table updates and exploration strategies

#### **Pathfinding & Navigation**
- [x] **A* Algorithm** - Optimal pathfinding with heuristic optimization
- [x] **Flood Fill (BFS)** - Guaranteed shortest path algorithm
- [x] **Waypoint System** - Intermediate navigation points for smooth movement
- [x] **Occupancy Grid** - Environment representation for pathfinding
- [x] **Hybrid Navigation** - Combines classical pathfinding with learning

### 🔄 **IN PROGRESS**

#### **Debugging & Optimization**
- [ ] **Episode Loop Bug Fix** - Investigating drone position corruption during episodes
- [ ] **Performance Tuning** - Optimizing reward functions and learning parameters
- [ ] **Stability Improvements** - Reducing stuck detection false positives

### 🚧 **NEXT PRIORITIES (Q4 2024)**

#### **Main Simulator Loop**
- [ ] **Real-time Simulation** - Complete main.cpp with UI event handling
- [ ] **Rendering Engine** - OpenCV-based visualization and debugging
- [ ] **Interactive Controls** - User input for testing and demonstration

#### **Vision Agent (Track B)**
- [ ] **ONNX Runtime Integration** - Neural network inference engine
- [ ] **Pre-trained Model Loading** - Model persistence and versioning
- [ ] **Real-time Inference** - Frame processing and action selection
- [ ] **Frame Stacking** - Temporal information for decision making

#### **Model Persistence**
- [ ] **Q-Table Save/Load** - Training progress preservation
- [ ] **Neural Network Persistence** - Vision model checkpointing
- [ ] **Configuration Management** - Hyperparameter storage and versioning

### 🌟 **FUTURE ENHANCEMENTS (2025)**

#### **Advanced Features**
- [ ] **Multi-Agent Simulation** - Multiple drones with coordination
- [ ] **Dynamic Obstacles** - Moving obstacles and adaptive pathfinding
- [ ] **3D Environment** - Elevation and vertical navigation
- [ ] **Real-time Learning** - Continuous adaptation during operation

#### **Deployment & Integration**
- [ ] **ROS Integration** - Robot Operating System compatibility
- [ ] **Hardware Interface** - Real drone control and sensor integration
- [ ] **Cloud Training** - Distributed learning and model sharing
- [ ] **Performance Benchmarks** - Comprehensive testing and optimization

## 🏗️ Architecture

The project follows a modular architecture with clear separation of concerns:

```
DronePathfindingAI/
├── sim/           # Simulation engine (Drone, World)
├── agent/         # Learning agents (QLearning, Vision)
├── bridge/        # RL environment wrapper
├── ui/            # User interface and visualization
└── tests/         # Comprehensive test suite
```

## 🚀 Getting Started

### Prerequisites
- C++17 compatible compiler
- CMake 3.15+
- OpenCV 4.x
- Eigen3

### Build Instructions
```bash
mkdir build && cd build
cmake ..
make -j4
```

### Running Tests
```bash
# Core functionality tests
./bin/minimal_test
./bin/environment_test
./bin/pathfinding_test

# Integration tests
./bin/episode_test
./bin/replicate_original
```

## 📊 Project Statistics

- **Total Source Files**: 25 (C++ headers and implementations)
- **Core Components**: 8 major classes
- **Test Coverage**: 6 comprehensive test suites
- **Build System**: CMake with proper dependency management
- **Language**: Primarily C++17 (GitHub now correctly shows ~85% C++)

## 🤝 Contributing

This is a research and educational project. Contributions are welcome in the form of:
- Bug reports and fixes
- Performance improvements
- Additional learning algorithms
- Documentation enhancements

## 📄 License

This project is open source and available under the MIT License.
