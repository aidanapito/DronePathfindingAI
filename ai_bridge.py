#!/usr/bin/env python3
"""
Communication Bridge for C++ Drone Simulation and Python AI
This script handles the communication between the C++ simulation and Python AI.
"""

import json
import time
import os
import sys
from pathlib import Path
from drone_ai import PathfindingAI, DroneState, DroneInput, Obstacle, AIMode

class AIBridge:
    """Bridge between C++ simulation and Python AI"""
    
    def __init__(self, data_dir="ai_data"):
        self.ai = PathfindingAI()
        self.data_dir = Path(data_dir)
        self.data_dir.mkdir(exist_ok=True)
        
        # File paths for communication
        self.drone_state_file = self.data_dir / "drone_state.json"
        self.obstacles_file = self.data_dir / "obstacles.json"
        self.ai_input_file = self.data_dir / "ai_input.json"
        self.command_file = self.data_dir / "command.json"
        
        print("🤖 Python AI Bridge initialized")
        print(f"📁 Data directory: {self.data_dir}")
        print("🔄 Waiting for C++ simulation to start...")
    
    def read_drone_state(self) -> DroneState:
        """Read drone state from JSON file"""
        try:
            if self.drone_state_file.exists():
                with open(self.drone_state_file, 'r') as f:
                    data = json.load(f)
                    return DroneState(
                        x=data['x'], y=data['y'], z=data['z'],
                        vx=data['vx'], vy=data['vy'], vz=data['vz'],
                        roll=data['roll'], pitch=data['pitch'], yaw=data['yaw']
                    )
        except Exception as e:
            print(f"❌ Error reading drone state: {e}")
        
        # Return default state if file doesn't exist or error
        return DroneState(0, 0, 0, 0, 0, 0, 0, 0, 0)
    
    def read_obstacles(self) -> list[Obstacle]:
        """Read obstacles from JSON file"""
        try:
            if self.obstacles_file.exists():
                with open(self.obstacles_file, 'r') as f:
                    data = json.load(f)
                    obstacles = []
                    for obs_data in data:
                        obstacles.append(Obstacle(
                            x=obs_data['x'], y=obs_data['y'], z=obs_data['z'],
                            width=obs_data['width'], height=obs_data['height'], 
                            radius=obs_data['radius']
                        ))
                    return obstacles
        except Exception as e:
            print(f"❌ Error reading obstacles: {e}")
        
        return []
    
    def read_command(self) -> dict:
        """Read command from JSON file"""
        try:
            if self.command_file.exists():
                with open(self.command_file, 'r') as f:
                    return json.load(f)
        except Exception as e:
            print(f"❌ Error reading command: {e}")
        
        return {}
    
    def write_ai_input(self, ai_input: DroneInput):
        """Write AI input to JSON file"""
        try:
            data = {
                'forward_thrust': ai_input.forward_thrust,
                'yaw_rate': ai_input.yaw_rate,
                'pitch_rate': ai_input.pitch_rate,
                'roll_rate': ai_input.roll_rate,
                'vertical_thrust': ai_input.vertical_thrust
            }
            with open(self.ai_input_file, 'w') as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            print(f"❌ Error writing AI input: {e}")
    
    def process_command(self, command: dict):
        """Process command from C++ simulation"""
        if 'mode' in command:
            mode_value = command['mode']
            if mode_value == 0:
                self.ai.set_mode(AIMode.MANUAL)
            elif mode_value == 1:
                self.ai.set_mode(AIMode.FOLLOW_PATH)
            elif mode_value == 2:
                self.ai.set_mode(AIMode.EXPLORE)
            elif mode_value == 3:
                self.ai.set_mode(AIMode.RETURN_HOME)
            elif mode_value == 4:
                self.ai.set_mode(AIMode.AVOID_OBSTACLES)
        
        if 'target' in command:
            target = command['target']
            self.ai.set_target(target['x'], target['y'], target['z'])
    
    def run(self):
        """Main loop - continuously read data and provide AI responses"""
        print("🚀 Starting AI bridge main loop...")
        
        while True:
            try:
                # Read current state from C++ simulation
                drone_state = self.read_drone_state()
                obstacles = self.read_obstacles()
                
                # Read any commands
                command = self.read_command()
                if command:
                    self.process_command(command)
                    # Clear the command file after processing
                    if self.command_file.exists():
                        self.command_file.unlink()
                
                # Update AI and get inputs
                ai_input = self.ai.update(0.016, drone_state, obstacles)  # Assume 60 FPS
                
                # Write AI input back to C++ simulation
                self.write_ai_input(ai_input)
                
                # Small delay to prevent excessive CPU usage
                time.sleep(0.01)
                
            except KeyboardInterrupt:
                print("\n🛑 AI bridge stopped by user")
                break
            except Exception as e:
                print(f"❌ Error in AI bridge loop: {e}")
                time.sleep(0.1)

def main():
    """Main function"""
    print("Python AI Bridge for C++ Drone Simulation")
    print("=" * 50)
    
    # Parse command line arguments
    data_dir = "ai_data"
    if len(sys.argv) > 1:
        data_dir = sys.argv[1]
    
    # Create and run the bridge
    bridge = AIBridge(data_dir)
    bridge.run()

if __name__ == "__main__":
    main()
