#!/usr/bin/env python3
"""
Simple AI Bridge for C++ Drone Simulation
This script reads data from the C++ simulation and provides AI inputs.
"""

import json
import time
import os
from pathlib import Path
from drone_ai import PathfindingAI, DroneState, DroneInput, AIMode

def main():
    print("🤖 Simple Python AI Bridge for C++ Drone Simulation")
    print("=" * 60)
    
    # Create AI instance
    ai = PathfindingAI()
    ai.set_mode(AIMode.FOLLOW_PATH)
    
    # Data directory
    data_dir = Path("ai_data")
    data_dir.mkdir(exist_ok=True)
    
    # File paths
    drone_state_file = data_dir / "drone_state.json"
    obstacles_file = data_dir / "obstacles.json"
    command_file = data_dir / "command.json"
    ai_input_file = data_dir / "ai_input.json"
    
    print(f"📁 Monitoring data directory: {data_dir}")
    print("🔄 Waiting for C++ simulation data...")
    print("💡 Start your C++ simulation and press 'T' then '2' to enable AI")
    
    while True:
        try:
            # Check for command file
            if command_file.exists():
                print(f"📥 Found command.json file")
                with open(command_file, 'r') as f:
                    command_data = json.load(f)
                
                # Process command
                mode = command_data.get('mode', 0)
                print(f"🔧 Processing command with mode: {mode}")
                if mode == 1:  # FOLLOW_PATH
                    target = command_data.get('target', {})
                    ai.set_target(target.get('x', 0), target.get('y', 0), target.get('z', 0))
                    print(f"🎯 Target set to: ({target.get('x', 0)}, {target.get('y', 0)}, {target.get('z', 0)})")
                
                # Remove command file
                command_file.unlink()
                print(f"🗑️ Deleted command.json file")
            
            # Check for drone state file
            if drone_state_file.exists():
                print(f"📥 Found drone_state.json file")
                with open(drone_state_file, 'r') as f:
                    drone_data = json.load(f)
                
                # Create drone state
                drone_state = DroneState(
                    x=drone_data['x'], y=drone_data['y'], z=drone_data['z'],
                    vx=drone_data['vx'], vy=drone_data['vy'], vz=drone_data['vz'],
                    roll=drone_data['roll'], pitch=drone_data['pitch'], yaw=drone_data['yaw']
                )
                
                # Read obstacles if available
                obstacles = []
                if obstacles_file.exists():
                    print(f"📥 Found obstacles.json file")
                    with open(obstacles_file, 'r') as f:
                        obstacles_data = json.load(f)
                        for obs_data in obstacles_data:
                            from drone_ai import Obstacle
                            obstacles.append(Obstacle(
                                x=obs_data['x'], y=obs_data['y'], z=obs_data['z'],
                                width=obs_data['width'], height=obs_data['height'], radius=obs_data['radius']
                            ))
                
                # Generate AI input
                print(f"🤖 Generating AI input...")
                ai_input = ai.update(0.016, drone_state, obstacles)
                
                # Write AI input back
                input_data = {
                    'forward_thrust': ai_input.forward_thrust,
                    'yaw_rate': ai_input.yaw_rate,
                    'pitch_rate': ai_input.pitch_rate,
                    'roll_rate': ai_input.roll_rate,
                    'vertical_thrust': ai_input.vertical_thrust
                }
                
                with open(ai_input_file, 'w') as f:
                    json.dump(input_data, f, indent=2)
                
                print(f"📤 AI Input written: F:{ai_input.forward_thrust:.3f} Y:{ai_input.yaw_rate:.3f} V:{ai_input.vertical_thrust:.3f}")
            else:
                print(f"⏳ Waiting for drone_state.json...")
            
            time.sleep(0.01)  # Small delay
            
        except KeyboardInterrupt:
            print("\n🛑 AI Bridge stopped by user")
            break
        except Exception as e:
            print(f"❌ Error: {e}")
            time.sleep(0.1)

if __name__ == "__main__":
    main()
