# Python AI for C++ Drone Simulation

This directory contains a Python-based AI system that can control your existing C++ drone simulation. The Python AI provides better debugging capabilities and easier development compared to the C++ AI.

## Files

- `drone_ai.py` - Main AI controller with pathfinding logic
- `ai_bridge.py` - Communication bridge between C++ and Python
- `test_ai.py` - Test script to verify AI functionality
- `requirements.txt` - Python dependencies

## Setup

1. **Install Python dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

2. **Test the AI system:**
   ```bash
   python test_ai.py
   ```

## How to Use with C++ Simulation

### Option 1: File-based Communication (Recommended)

The AI bridge uses JSON files to communicate with the C++ simulation:

1. **Start the Python AI bridge:**
   ```bash
   python ai_bridge.py
   ```

2. **Modify your C++ simulation** to:
   - Write drone state to `ai_data/drone_state.json`
   - Write obstacles to `ai_data/obstacles.json`
   - Read AI inputs from `ai_data/ai_input.json`
   - Send commands via `ai_data/command.json`

### Option 2: Direct Integration

You can also integrate the Python AI directly into your C++ code using:
- `pybind11` for direct Python embedding
- Network sockets for real-time communication
- Shared memory for high-performance communication

## Communication Protocol

### C++ → Python (JSON files)

**Drone State (`drone_state.json`):**
```json
{
  "x": 100.0,
  "y": 200.0,
  "z": 50.0,
  "vx": 0.0,
  "vy": 0.0,
  "vz": 0.0,
  "roll": 0.0,
  "pitch": 0.0,
  "yaw": 0.0
}
```

**Obstacles (`obstacles.json`):**
```json
[
  {
    "x": 150.0,
    "y": 150.0,
    "z": 0.0,
    "width": 20.0,
    "height": 30.0,
    "radius": 15.0
  }
]
```

**Commands (`command.json`):**
```json
{
  "mode": 1,
  "target": {
    "x": 300.0,
    "y": 300.0,
    "z": 80.0
  }
}
```

### Python → C++ (JSON file)

**AI Input (`ai_input.json`):**
```json
{
  "forward_thrust": 0.5,
  "yaw_rate": 0.2,
  "pitch_rate": 0.1,
  "roll_rate": 0.0,
  "vertical_thrust": 0.3
}
```

## AI Modes

- `MANUAL (0)` - No AI control
- `FOLLOW_PATH (1)` - Follow path to target
- `EXPLORE (2)` - Explore environment
- `RETURN_HOME (3)` - Return to starting position
- `AVOID_OBSTACLES (4)` - Avoid obstacles while moving

## Advantages of Python AI

1. **Better Debugging** - Rich debug output and interactive debugging
2. **Easier Development** - Python's dynamic typing and rich ecosystem
3. **Visualization** - Can easily add plotting and visualization
4. **Rapid Prototyping** - Faster iteration and testing
5. **AI Libraries** - Access to NumPy, SciPy, and ML libraries

## Next Steps

1. **Test the Python AI** with `python test_ai.py`
2. **Modify your C++ simulation** to use the file-based communication
3. **Start the AI bridge** with `python ai_bridge.py`
4. **Run your C++ simulation** and watch the Python AI control it!

## Troubleshooting

- **Import errors**: Make sure all dependencies are installed
- **File not found**: Check that the `ai_data` directory exists
- **Communication issues**: Verify JSON file formats match the expected structure
