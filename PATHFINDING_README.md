# Pathfinding and Q-Learning Integration

This document describes the implementation of basic pathfinding algorithms and their integration with the Q-Learning agent for drone navigation.

## Overview

The system now combines two powerful approaches:
1. **Classical Pathfinding**: A* and Flood Fill algorithms for optimal path computation
2. **Q-Learning**: Reinforcement learning for adaptive navigation and obstacle avoidance

## Pathfinding Algorithms

### A* Algorithm
- **Implementation**: `World::findPathAStar()`
- **Features**: 
  - Optimal path finding with heuristic-based search
  - Manhattan distance heuristic
  - 8-directional movement
  - Grid-based discretization
- **Use Case**: When optimal paths are required

### Flood Fill (BFS)
- **Implementation**: `World::findPathFloodFill()`
- **Features**:
  - Breadth-first search for shortest path
  - Guaranteed to find path if one exists
  - Simpler than A* but may be less optimal
- **Use Case**: When guaranteed path finding is needed

## Q-Learning Integration

### Core Q-Learning Features
- **State Space**: Grid position + heading discretization
- **Action Space**: Forward, Left Turn, Right Turn, Idle
- **Learning**: Q-value updates with exploration vs exploitation
- **Stuck Detection**: Advanced loop detection and escape mechanisms

### Pathfinding Integration
- **Hybrid Approach**: Combines classical pathfinding with Q-Learning
- **Waypoint Following**: Q-Learning agent follows computed optimal path
- **Adaptive Navigation**: Learns to handle dynamic obstacles and deviations
- **Reward Shaping**: Path following rewards + traditional Q-Learning rewards

## Architecture

```
Environment
├── World (Pathfinding)
│   ├── A* Algorithm
│   ├── Flood Fill
│   └── Occupancy Grid
├── QLearningAgent
│   ├── Q-Table
│   ├── Stuck Detection
│   └── Path Integration
└── Drone (Execution)
```

## Key Components

### Environment Class
- **Pathfinding Control**: Enable/disable pathfinding, select algorithm
- **Path Integration**: Computes optimal path and tracks waypoints
- **Reward Shaping**: Combines path following with Q-Learning rewards

### World Class
- **Path Computation**: Implements A* and Flood Fill algorithms
- **Grid Management**: Handles world discretization and collision detection
- **Obstacle Avoidance**: Integrates with pathfinding for valid paths

### QLearningAgent Class
- **Path Awareness**: Receives optimal path from Environment
- **Waypoint Navigation**: Uses path information for better action selection
- **Learning Integration**: Combines classical and learned navigation

## Usage

### Basic Setup
```cpp
// Create world and agent
auto world = std::make_shared<sim::World>(800, 600);
auto agent = std::make_shared<agent::QLearningAgent>(config);

// Create environment with pathfinding
auto env = std::make_shared<bridge::Environment>(env_config);
env->setUsePathfinding(true);
env->setPathfindingAlgorithm("astar"); // or "floodfill"
```

### Running Episodes
```cpp
// Environment automatically computes optimal path
auto result = env->runEpisode(agent);

// Access path information
auto optimal_path = env->getOptimalPath();
std::cout << "Path has " << optimal_path.size() << " waypoints" << std::endl;
```

### Testing
```bash
# Build the project
mkdir build && cd build
cmake ..
make

# Run pathfinding test
./bin/test_pathfinding
```

## Configuration Options

### Pathfinding
- **Algorithm**: "astar" or "floodfill"
- **Grid Size**: Configurable discretization (default: 10.0f)
- **Enable/Disable**: Toggle pathfinding integration

### Q-Learning
- **Learning Rate**: Alpha parameter for Q-value updates
- **Discount Factor**: Gamma for future reward consideration
- **Exploration Rate**: Epsilon for exploration vs exploitation
- **Stuck Detection**: Thresholds for loop detection and escape

## Benefits

1. **Optimal Paths**: Classical algorithms provide guaranteed optimal solutions
2. **Adaptive Learning**: Q-Learning handles dynamic obstacles and deviations
3. **Robust Navigation**: Multiple fallback mechanisms for stuck situations
4. **Efficient Learning**: Path guidance reduces random exploration
5. **Scalable**: Grid-based approach scales with world size

## Future Enhancements

1. **Dynamic Pathfinding**: Real-time path updates for moving obstacles
2. **Multi-Agent Coordination**: Path planning for multiple drones
3. **Advanced Heuristics**: Machine learning-based heuristic functions
4. **3D Navigation**: Extension to 3D space with height considerations
5. **Real-time Optimization**: Continuous path optimization during execution

## Performance Considerations

- **Grid Resolution**: Smaller grids = more precise paths but higher computation
- **Path Updates**: Recompute paths when obstacles move significantly
- **Memory Usage**: Q-table size grows with state space discretization
- **Learning Convergence**: Balance exploration with path following

## Troubleshooting

### Common Issues
1. **No Path Found**: Check obstacle density and grid resolution
2. **Poor Learning**: Adjust learning rate and exploration parameters
3. **Stuck Behavior**: Review stuck detection thresholds
4. **Memory Issues**: Reduce grid resolution or heading discretization

### Debug Information
- Use `agent->getDebugInfo()` for detailed agent state
- Check `env->getOptimalPath()` for path computation results
- Monitor waypoint progress in environment logs
