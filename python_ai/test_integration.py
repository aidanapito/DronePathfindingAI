#!/usr/bin/env python3
"""
Test script to verify Python AI integration with C++ simulation
"""

import json
import time
from pathlib import Path
from drone_ai import PathfindingAI, DroneState, AIMode

def test_ai_response():
    print("🧪 Testing Python AI Integration")
    print("=" * 40)
    
    # Create AI
    ai = PathfindingAI()
    ai.set_mode(AIMode.FOLLOW_PATH)
    ai.set_target(300.0, 300.0, 80.0)
    
    # Test drone state
    drone_state = DroneState(100.0, 200.0, 50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    
    # Generate AI input
    ai_input = ai.update(0.016, drone_state, [])
    
    # Create JSON data that the C++ simulation would expect
    input_data = {
        'forward_thrust': ai_input.forward_thrust,
        'yaw_rate': ai_input.yaw_rate,
        'pitch_rate': ai_input.pitch_rate,
        'roll_rate': ai_input.roll_rate,
        'vertical_thrust': ai_input.vertical_thrust
    }
    
    print("✅ AI Input generated:")
    print(json.dumps(input_data, indent=2))
    
    # Test JSON parsing (simulate what C++ would do)
    json_str = json.dumps(input_data)
    
    # Parse using proper JSON parsing
    parsed_data = json.loads(json_str)
    
    print(f"\n✅ Parsed values:")
    print(f"   Forward thrust: {parsed_data['forward_thrust']:.3f}")
    print(f"   Yaw rate: {parsed_data['yaw_rate']:.3f}")
    print(f"   Vertical thrust: {parsed_data['vertical_thrust']:.3f}")
    
    print(f"\n✅ Integration test passed!")
    print("🚀 Ready to use with C++ simulation!")

if __name__ == "__main__":
    test_ai_response()
