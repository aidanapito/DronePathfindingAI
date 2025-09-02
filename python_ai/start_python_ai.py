#!/usr/bin/env python3
"""
Start script for Python AI Bridge
This script starts the Python AI bridge that will communicate with the C++ simulation.
"""

import subprocess
import sys
import time
import os
from pathlib import Path

def main():
    print("🚀 Starting Python AI Bridge for C++ Drone Simulation")
    print("=" * 60)
    
    # Check if we're in the right directory
    if not Path("drone_ai.py").exists():
        print("❌ Error: drone_ai.py not found!")
        print("Please run this script from the python_ai directory")
        sys.exit(1)
    
    # Create ai_data directory
    data_dir = Path("ai_data")
    data_dir.mkdir(exist_ok=True)
    print(f"📁 Created data directory: {data_dir}")
    
    print("\n🤖 Starting Python AI Bridge...")
    print("📋 Instructions:")
    print("1. Keep this terminal open")
    print("2. In another terminal, run: cd ../build && ./drone_sim")
    print("3. Press 'T' then '2' to enable AI mode")
    print("4. Watch the Python AI control the drone!")
    print("\n" + "=" * 60)
    
    try:
        # Import and run the AI bridge
        from drone_ai import PathfindingAI, DroneState, AIMode
        
        # Create AI instance
        ai = PathfindingAI()
        ai.set_mode(AIMode.FOLLOW_PATH)
        
        print("✅ Python AI initialized successfully!")
        print("🔄 Waiting for C++ simulation to start...")
        
        # Simple test to show AI is working
        test_drone = DroneState(100.0, 200.0, 50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        ai.set_target(300.0, 300.0, 80.0)
        
        print("\n🧪 Testing AI with sample data:")
        ai_input = ai.update(0.016, test_drone, [])
        print(f"   Forward thrust: {ai_input.forward_thrust:.3f}")
        print(f"   Yaw rate: {ai_input.yaw_rate:.3f}")
        print(f"   Vertical thrust: {ai_input.vertical_thrust:.3f}")
        
        print("\n✅ AI is ready! Start your C++ simulation now.")
        print("Press Ctrl+C to stop the AI bridge")
        
        # Keep the script running
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n🛑 Python AI Bridge stopped by user")
    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
