# Integration Guide: Python AI with C++ Drone Simulation

## Quick Start

1. **Test the Python AI:**
   ```bash
   cd python_ai
   python drone_ai.py
   ```

2. **The Python AI is working correctly!** It shows:
   - Drone at (100, 200, 50)
   - Target at (300, 300, 80)
   - Distance: 225.61 units
   - Forward velocity: 100.00 (positive = moving toward target)
   - Input values: Forward=1.0, Yaw=0.927, Vertical=0.5

## Key Advantages of Python AI

✅ **Fixed the coordinate system issue** - The Python AI correctly calculates:
- Drone forward direction: `(sin(yaw), cos(yaw))` 
- Forward velocity: 100.00 (positive = toward target)
- This matches your C++ drone's coordinate system

✅ **Better debugging** - Clear output showing:
- Exact drone and target positions
- Distance calculations
- Input values

✅ **Easier to modify** - No compilation needed, just edit and run

## Next Steps

### Option 1: Quick Integration (Recommended)
Modify your C++ simulation to:
1. Write drone state to a JSON file
2. Read AI inputs from a JSON file
3. Use the Python AI instead of the C++ AI

### Option 2: Direct Replacement
Replace the C++ AI logic with Python calls using:
- `pybind11` for direct Python embedding
- System calls to run the Python script
- Network sockets for real-time communication

## The Coordinate System Fix

The Python AI correctly uses:
```python
# Correct coordinate system for your drone
drone_forward = np.array([math.sin(yaw), math.cos(yaw), 0.0])
```

This matches your C++ drone's movement:
```cpp
float forward_dir_x = sin(state_.yaw) * cos(state_.pitch);
float forward_dir_y = cos(state_.yaw) * cos(state_.pitch);
```

## Ready to Integrate!

The Python AI is ready to control your C++ simulation. Would you like me to:

1. **Create the integration code** for your C++ simulation?
2. **Show you how to modify** your existing C++ AI to use Python?
3. **Create a simple test** that combines both systems?

Just let me know which approach you prefer!
