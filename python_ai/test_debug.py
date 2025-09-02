#!/usr/bin/env python3

import math
import numpy as np
from drone_ai import PathfindingAI, DroneState, AIMode

def test_exact_scenario():
    print("🧪 Testing exact scenario from simulation...")
    
    # Exact drone state from the simulation
    drone_state = DroneState(
        x=324.696686,
        y=560.540161, 
        z=89.999931,
        vx=0.000000,
        vy=-0.000385,
        vz=0.000211,
        roll=-0.000000,
        pitch=0.000000,
        yaw=6.283185
    )
    
    # Target from simulation
    target_x = 486.240448
    target_y = 560.540283
    target_z = 90.000000
    
    print(f"🎯 Drone: ({drone_state.x:.3f}, {drone_state.y:.3f}, {drone_state.z:.3f}) yaw: {drone_state.yaw:.3f} ({drone_state.yaw * 180 / math.pi:.1f}°)")
    print(f"🎯 Target: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})")
    
    # Create AI and test
    ai = PathfindingAI()
    ai.set_mode(AIMode.FOLLOW_PATH)
    ai.set_target(target_x, target_y, target_z)
    
    # Calculate manually
    drone_pos = np.array([drone_state.x, drone_state.y, drone_state.z])
    target_pos = np.array([target_x, target_y, target_z])
    to_target = target_pos - drone_pos
    
    print(f"🎯 To target: ({to_target[0]:.3f}, {to_target[1]:.3f}, {to_target[2]:.3f})")
    print(f"🎯 Distance: {np.linalg.norm(to_target):.3f}")
    
    # Calculate target direction
    target_direction = to_target / (np.linalg.norm(to_target) + 1e-6)
    target_yaw = math.atan2(target_direction[1], target_direction[0])
    yaw_error = target_yaw - drone_state.yaw
    
    # Normalize yaw error
    while yaw_error > math.pi:
        yaw_error -= 2 * math.pi
    while yaw_error < -math.pi:
        yaw_error += 2 * math.pi
    
    print(f"🎯 Target direction: ({target_direction[0]:.3f}, {target_direction[1]:.3f}, {target_direction[2]:.3f})")
    print(f"🎯 Target yaw: {target_yaw:.3f} ({target_yaw * 180 / math.pi:.1f}°)")
    print(f"🎯 Current yaw: {drone_state.yaw:.3f} ({drone_state.yaw * 180 / math.pi:.1f}°)")
    print(f"🎯 Yaw error: {yaw_error:.3f} ({yaw_error * 180 / math.pi:.1f}°)")
    
    # Calculate drone forward direction
    drone_forward = np.array([math.sin(drone_state.yaw), math.cos(drone_state.yaw), 0.0])
    forward_velocity = np.dot(to_target, drone_forward)
    
    print(f"🎯 Drone forward: ({drone_forward[0]:.3f}, {drone_forward[1]:.3f}, {drone_forward[2]:.3f})")
    print(f"🎯 Forward velocity: {forward_velocity:.3f}")
    
    # Calculate inputs
    forward_thrust = np.clip(forward_velocity / 10.0, -1.0, 1.0)
    vertical_thrust = np.clip(to_target[2] / 10.0, -0.5, 0.5)
    yaw_rate = np.clip(yaw_error * 5.0, -1.0, 1.0)
    
    print(f"🎯 Calculated inputs - F:{forward_thrust:.6f} Y:{yaw_rate:.6f} V:{vertical_thrust:.6f}")
    
    # Test AI
    ai_input = ai.update(0.016, drone_state)
    print(f"🎯 AI inputs - F:{ai_input.forward_thrust:.6f} Y:{ai_input.yaw_rate:.6f} V:{ai_input.vertical_thrust:.6f}")

if __name__ == "__main__":
    test_exact_scenario()
