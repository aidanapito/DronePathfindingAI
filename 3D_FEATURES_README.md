# 3D Drone Pathfinding AI - New Features

## Overview
Your drone pathfinding AI system has been successfully extended to support full 3D movement and navigation! This includes 3D drone physics, 3D world generation, 3D pathfinding algorithms, and 3D visualization.

## What's New in 3D

### 1. 3D Drone Physics
- **3D Position**: Now uses `cv::Point3f` (x, y, z) instead of just `cv::Point2f` (x, y)
- **3D Orientation**: Added pitch and roll angles in addition to yaw (heading)
- **3D Movement**: Support for vertical thrust and 3D velocity
- **3D Constraints**: Altitude limits, vertical velocity limits, pitch/roll rate limits

### 2. 3D Actions
The drone can now perform these 3D maneuvers:
- **Basic 3D**: `PITCH_UP`, `PITCH_DOWN`, `ROLL_LEFT`, `ROLL_RIGHT`, `THRUST_UP`, `THRUST_DOWN`
- **Combined 3D**: `FORWARD_AND_UP`, `FORWARD_AND_DOWN`, `TURN_AND_CLIMB`, `TURN_AND_DIVE`
- **Backward Compatibility**: All original 2D actions still work

### 3. 3D World Generation
- **3D Obstacles**: Obstacles now have height and can extend vertically
- **New Map Types**: `SKYSCRAPER`, `UNDERWATER`, `MOUNTAIN_PASS`
- **3D Collision Detection**: Full 3D collision checking with height considerations
- **3D Occupancy Grids**: 21x21x10 3D occupancy grids for better spatial awareness

### 4. 3D Pathfinding
- **3D A* Algorithm**: Full 3D pathfinding with height considerations
- **3D Flood Fill**: Alternative 3D pathfinding method
- **Height-Aware Routing**: Paths can now go over or under obstacles
- **Vertical Waypoints**: Paths include altitude changes when beneficial

### 5. 3D Observations
- **3D Position Data**: Full 3D position, orientation, and velocity
- **Altitude Information**: Current altitude, goal altitude, altitude bounds
- **3D Clearance**: Distance to obstacles above, below, and forward
- **3D Occupancy Grids**: Multi-layer height-based occupancy information

### 6. 3D Visualization
- **3D Camera System**: Orbit, zoom, and pan camera controls
- **3D Rendering**: Full 3D scene rendering with OpenCV
- **Multiple View Modes**: 2D, 3D, Top-Down, First-Person
- **Interactive Controls**: Mouse and keyboard camera manipulation

## How to Use 3D Features

### Basic 3D Setup
```cpp
// Enable 3D mode in environment
bridge::EnvironmentConfig config;
config.enable_3d_mode = true;
config.enable_3d_pathfinding = true;
config.max_altitude = 300.0f;
config.min_altitude = 10.0f;

// Create 3D world
auto world = std::make_shared<sim::World>(800, 600, 400);
world->generateMap(sim::MapType::SKYSCRAPER);

// Create 3D drone
auto drone = std::make_shared<sim::Drone>(cv::Point3f(50, 50, 50), 0.0f, 0.0f, 0.0f);
```

### 3D Agent Configuration
```cpp
agent::AgentConfig agent_config;
agent_config.use_3d = true;
agent_config.max_altitude = 300.0f;
agent_config.min_altitude = 10.0f;
agent_config.enable_3d_pathfinding = true;
```

### 3D Visualization
```cpp
auto ui = std::make_shared<ui::SimulatorUI>();
ui->setRenderMode("3D");
ui->setShowGrid(true);
ui->setShowAxes(true);

// Set up 3D camera
ui::Camera3D camera;
camera.position = cv::Point3f(400, 300, 250);
camera.target = cv::Point3f(400, 300, 100);
ui->setCamera(camera);
```

## Running the 3D Demo

### Compilation
```bash
# Make sure you have OpenCV and Eigen installed
mkdir build && cd build
cmake ..
make 3d_demo
```

### Execution
```bash
./3d_demo
```

### Controls
- **Mouse Drag**: Rotate camera around target
- **Mouse Wheel**: Zoom in/out
- **Right Mouse Drag**: Pan camera
- **R**: Reset camera to default position
- **Space**: Pause/Resume simulation
- **ESC**: Exit

## 3D Map Types

### 1. SKYSCRAPER
- Urban environment with tall buildings
- Multiple height levels
- Good for testing vertical navigation

### 2. UNDERWATER
- Submerged environment with varying depths
- Floating obstacles at different heights
- Tests depth perception and vertical planning

### 3. MOUNTAIN_PASS
- Terrain with elevation changes
- Natural obstacles at various heights
- Good for testing 3D pathfinding

## 3D Reward System

The 3D system includes new reward components:
- **Altitude Reward**: Bonus for maintaining good altitude
- **Vertical Progress**: Reward for moving toward goal altitude
- **Clearance Reward**: Bonus for maintaining safe distances above/below
- **3D Path Following**: Reward for following 3D optimal paths

## Performance Considerations

### 3D Pathfinding
- 3D A* is more computationally intensive than 2D
- Grid resolution affects performance vs. accuracy trade-off
- Consider using `enable_3d_pathfinding = false` for faster 2D-only mode

### 3D Rendering
- 3D visualization requires more GPU resources
- Reduce frame rate if experiencing lag
- Use 2D mode for faster simulation without visualization

## Backward Compatibility

All existing 2D code will continue to work:
- 2D drone positions are automatically converted to 3D (z=0)
- 2D actions still function normally
- 2D pathfinding algorithms remain available
- 2D visualization modes are preserved

## Future Enhancements

Potential areas for further 3D development:
- **Physics Engine**: More realistic 3D drone physics
- **Terrain Generation**: Procedural 3D terrain
- **Multi-Drone**: 3D swarm simulation
- **Advanced Sensors**: 3D LIDAR, depth cameras
- **VR Support**: Virtual reality visualization

## Troubleshooting

### Common Issues

1. **Compilation Errors**: Ensure OpenCV supports 3D operations
2. **Performance Issues**: Reduce 3D grid resolution or disable 3D pathfinding
3. **Visualization Problems**: Check OpenCV version compatibility
4. **Memory Issues**: Reduce world size or obstacle count

### Debug Mode
Enable debug output by setting environment variables:
```bash
export DRONE_DEBUG=1
export DRONE_3D_DEBUG=1
```

## Examples

See the following files for complete examples:
- `src/3d_demo.cpp` - Full 3D simulation demo
- `src/main.cpp` - Updated main program with 3D support
- `test_pathfinding.cpp` - 3D pathfinding tests

## Contributing

When adding new 3D features:
1. Maintain backward compatibility
2. Add appropriate unit tests
3. Update documentation
4. Consider performance implications
5. Test with different map types

---

**Enjoy your new 3D drone simulation!** 🚁✨
