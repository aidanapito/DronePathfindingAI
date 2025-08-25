# Drone Pathfinding AI

A self-contained C++17 project implementing intelligent drone navigation using reinforcement learning, featuring both discrete Q-learning and vision-based deep learning approaches.

## 🚁 Project Overview

This project implements a lightweight drone simulator with two learning tracks:

- **Track A (Baseline)**: Discrete grid-based Q-learning for navigation
- **Track B (Vision RL)**: Raw image input with DQN/PPO inference using ONNX Runtime or LibTorch

The system provides a complete simulation environment with procedurally generated maps, obstacle avoidance, and real-time learning capabilities.

## 🏗️ System Architecture

```
DronePathfindingAI/
├── include/                    # Header files
│   ├── sim/                   # Simulation components
│   │   ├── World.h           # World generation & obstacle management
│   │   └── Drone.h           # Drone kinematics & sensors
│   ├── agent/                 # Learning agents
│   │   ├── Agent.h           # Base agent interface
│   │   ├── QLearningAgent.h  # Track A: Q-learning implementation
│   │   └── VisionAgent.h     # Track B: Vision-based agent
│   ├── bridge/                # Environment bridge
│   │   └── Environment.h     # RL environment wrapper
│   ├── ui/                    # User interface
│   │   └── SimulatorUI.h     # OpenCV rendering & controls
│   └── Simulator.h            # Main simulator orchestration
├── src/                       # Source files
│   ├── sim/                   # Simulation implementations
│   ├── agent/                 # Agent implementations
│   ├── bridge/                # Bridge implementations
│   ├── ui/                    # UI implementations
│   └── main.cpp              # Application entry point
├── CMakeLists.txt             # Build configuration
└── README.md                  # This file
```

## ✨ Key Features

- **Lightweight Simulator**: Top-down camera feed with OpenCV rendering
- **Procedural Maps**: Random mazes, corridors, open fields, and obstacle courses
- **Moving Obstacles**: Dynamic environment with collision detection
- **Dual Learning Tracks**: Q-learning (discrete) and vision-based (continuous) approaches
- **Safety Layer**: Hard constraints and emergency stop mechanisms
- **Real-time UI**: Interactive controls, path visualization, and video recording
- **Cross-platform**: Runs on macOS and Windows

## 🛠️ Technology Stack

- **C++17**: Modern C++ with smart pointers and standard library
- **OpenCV**: Computer vision, rendering, and video I/O
- **Eigen3**: Linear algebra and mathematical operations
- **CMake**: Cross-platform build system
- **ONNX Runtime**: Model inference for Track B (optional)
- **LibTorch**: PyTorch C++ API alternative (optional)

## 🚀 Getting Started

### Prerequisites

- **C++17 compatible compiler** (GCC 7+, Clang 5+, MSVC 2017+)
- **CMake 3.17+**
- **OpenCV 4.5+**
- **Eigen3 3.3+**

### Optional Dependencies

- **ONNX Runtime 1.8+** (for Track B vision inference)
- **LibTorch 1.9+** (alternative to ONNX Runtime)

### Installation

#### macOS (using Homebrew)

```bash
# Install dependencies
brew install opencv eigen cmake

# Optional: ONNX Runtime
brew install onnxruntime

# Clone and build
git clone https://github.com/aidanapito/DronePathfindingAI.git
cd DronePathfindingAI
mkdir build && cd build
cmake ..
make -j$(nproc)
```

#### Windows (using vcpkg)

```bash
# Install vcpkg if not already installed
git clone https://github.com/Microsoft/vcpkg.git
cd vcpkg
./bootstrap-vcpkg.bat

# Install dependencies
vcpkg install opencv eigen3 cmake

# Clone and build
git clone https://github.com/aidanapito/DronePathfindingAI.git
cd DronePathfindingAI
mkdir build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=[path_to_vcpkg]/scripts/buildsystems/vcpkg.cmake
cmake --build . --config Release
```

### Building

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)  # or cmake --build . on Windows
```

## 🎮 Usage

### Basic Simulation

```bash
# Run with default settings (Track A, maze map)
./DronePathfindingAI

# Use vision-based learning (Track B)
./DronePathfindingAI --vision

# Different map types
./DronePathfindingAI --corridor
./DronePathfindingAI --open
./DronePathfindingAI --obstacle
```

### Controls

- **P**: Pause/Resume simulation
- **R**: Reset current episode
- **N**: Generate new map
- **V**: Start/Stop video recording
- **ESC**: Quit application

### Learning Tracks

#### Track A: Q-Learning (Discrete)
- **State Space**: 21×21 occupancy grid + quantized heading (8 buckets)
- **Action Space**: {forward, left, right, idle}
- **Learning**: Q-table updates with experience replay
- **Advantages**: Fast convergence, interpretable policies
- **Use Case**: Baseline performance, rapid prototyping

#### Track B: Vision-Based RL (Continuous)
- **State Space**: 84×84 grayscale frames (stacked 4 frames)
- **Action Space**: Same as Track A
- **Learning**: Pre-trained DQN/PPO models exported to ONNX
- **Advantages**: Rich visual features, better generalization
- **Use Case**: Production deployment, complex environments

## 🔬 Technical Details

### Reward Function

```
R = +1.0  (goal reached)
    -1.0  (collision)
    +0.02 (positive progress)
    -0.005 (time penalty)
    -0.02 (safety margin violation)
```

### Safety Constraints

- **Turn Rate Limit**: Maximum angular velocity constraint
- **Emergency Stop**: Automatic stop when obstacle < safety distance
- **Collision Detection**: Real-time obstacle avoidance
- **Boundary Checking**: World boundary enforcement

### Domain Randomization

- **Lighting**: Brightness and contrast variations
- **Textures**: Background texture randomization
- **Obstacle Patterns**: Procedural generation with seed control
- **Moving Objects**: Dynamic obstacle trajectories

## 📊 Performance Metrics

- **Success Rate**: Percentage of successful navigation episodes
- **Average Episode Length**: Mean steps per episode
- **Collision Rate**: Safety performance measurement
- **Inference Time**: Model prediction latency (Track B)
- **FPS**: Real-time simulation performance

## 🔧 Configuration

The simulator can be configured through the `SimulatorConfig` struct:

```cpp
struct SimulatorConfig {
    int window_width = 1200;
    int window_height = 800;
    float target_fps = 60.0f;
    float time_step = 1.0f / 60.0f;
    bool use_vision_track = false;
    float learning_rate = 0.001f;
    float discount_factor = 0.99f;
    float epsilon = 0.1f;
    sim::MapType default_map_type = sim::MapType::MAZE;
};
```

## 🧪 Training

### Track A: Q-Learning
- **Environment**: Built-in C++ implementation
- **Hyperparameters**: Configurable learning rate, discount factor, epsilon
- **Experience**: Automatic replay buffer management
- **Persistence**: Q-table save/load functionality

### Track B: Vision-Based
- **Training**: Python environment (PyTorch/TensorFlow)
- **Export**: ONNX format for C++ inference
- **Models**: DQN, PPO, or custom architectures
- **Integration**: Seamless C++ deployment

## 🐛 Troubleshooting

### Common Issues

1. **OpenCV not found**: Ensure OpenCV is properly installed and CMake can find it
2. **Eigen3 missing**: Install Eigen3 development headers
3. **ONNX Runtime errors**: Check version compatibility and installation
4. **Build failures**: Verify C++17 compiler support

### Debug Mode

```bash
# Enable debug output
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j$(nproc)
```

## 🤝 Contributing

We welcome contributions! Please see our contributing guidelines:

1. Fork the repository
2. Create a feature branch
3. Implement your changes
4. Add tests if applicable
5. Submit a pull request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- OpenCV community for computer vision tools
- Eigen developers for linear algebra library
- ONNX Runtime team for model inference
- Reinforcement learning research community

## 📞 Support

- **Issues**: [GitHub Issues](https://github.com/aidanapito/DronePathfindingAI/issues)
- **Discussions**: [GitHub Discussions](https://github.com/aidanapito/DronePathfindingAI/discussions)
- **Wiki**: [Project Wiki](https://github.com/aidanapito/DronePathfindingAI/wiki)

---

**Note**: This is a research and educational project. Please ensure all safety protocols are followed when testing with real drones.
