#!/usr/bin/env python3
"""
Test script for the Python AI system
This script tests the AI functionality independently.
"""

import numpy as np
from drone_ai import PathfindingAI, DroneState, Obstacle, AIMode

def test_ai():
    """Test the AI system with a simple scenario"""
    print("🧪 Testing Python AI System")
    print("=" * 40)
    
    # Create AI instance
    ai = PathfindingAI()
    
    # Create test drone state
    drone_state = DroneState(
        x=100.0, y=200.0, z=50.0,
        vx=0.0, vy=0.0, vz=0.0,
        roll=0.0, pitch=0.0, yaw=0.0
    )
    
    # Create test obstacles
    obstacles = [
        Obstacle(x=150.0, y=150.0, z=0.0, width=20.0, height=30.0, radius=15.0),
        Obstacle(x=200.0, y=200.0, z=0.0, width=25.0, height=40.0, radius=20.0),
    ]
    
    # Set AI mode and target
    ai.set_mode(AIMode.FOLLOW_PATH)
    ai.set_target(300.0, 300.0, 80.0)
    
    print(f"🎯 Target: ({ai.target_position[0]}, {ai.target_position[1]}, {ai.target_position[2]})")
    print(f"🤖 AI Mode: {ai.current_mode.name}")
    print(f"📊 AI State: {ai.current_state.name}")
    
    # Test AI update
    print("\n🔄 Testing AI update...")
    ai_input = ai.update(0.016, drone_state, obstacles)
    
    print(f"📤 AI Input:")
    print(f"   Forward thrust: {ai_input.forward_thrust:.3f}")
    print(f"   Yaw rate: {ai_input.yaw_rate:.3f}")
    print(f"   Pitch rate: {ai_input.pitch_rate:.3f}")
    print(f"   Roll rate: {ai_input.roll_rate:.3f}")
    print(f"   Vertical thrust: {ai_input.vertical_thrust:.3f}")
    
    print("\n✅ Test completed successfully!")

if __name__ == "__main__":
    test_ai()
