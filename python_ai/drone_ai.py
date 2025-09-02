#!/usr/bin/env python3
"""
Simple Python AI for C++ Drone Simulation
"""

import numpy as np
import math
from dataclasses import dataclass
from enum import Enum

class AIMode(Enum):
    MANUAL = 0
    FOLLOW_PATH = 1
    EXPLORE = 2
    RETURN_HOME = 3
    AVOID_OBSTACLES = 4

class AIState(Enum):
    IDLE = 0
    PLANNING_PATH = 1
    FOLLOWING_PATH = 2
    AVOIDING_OBSTACLE = 3
    COMPLETED_MISSION = 4
    ERROR = 5

@dataclass
class DroneState:
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    roll: float
    pitch: float
    yaw: float

@dataclass
class DroneInput:
    forward_thrust: float = 0.0
    yaw_rate: float = 0.0
    pitch_rate: float = 0.0
    roll_rate: float = 0.0
    vertical_thrust: float = 0.0

@dataclass
class Obstacle:
    x: float
    y: float
    z: float
    width: float
    height: float
    radius: float

class PathfindingAI:
    def __init__(self):
        self.current_mode = AIMode.MANUAL
        self.current_state = AIState.IDLE
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.debug_enabled = True
        
    def set_mode(self, mode: AIMode):
        self.current_mode = mode
        if self.debug_enabled:
            print(f"🤖 AI Mode set to: {mode.name}")
    
    def set_target(self, x: float, y: float, z: float):
        self.target_position = np.array([x, y, z])
        if self.debug_enabled:
            print(f"🎯 Target set to: ({x}, {y}, {z})")
    
    def update(self, delta_time: float, drone_state: DroneState, obstacles=None) -> DroneInput:
        if obstacles is None:
            obstacles = []
        if self.current_mode == AIMode.MANUAL:
            return DroneInput()
        
        elif self.current_mode == AIMode.FOLLOW_PATH:
            return self._generate_path_following_input(drone_state)
        
        return DroneInput()
    
    def _generate_path_following_input(self, drone_state: DroneState) -> DroneInput:
        drone_pos = np.array([drone_state.x, drone_state.y, drone_state.z])
        to_target = self.target_position - drone_pos
        
        if self.debug_enabled:
            print(f"🎯 Drone at: ({drone_pos[0]:.2f}, {drone_pos[1]:.2f}, {drone_pos[2]:.2f})")
            print(f"🎯 Target at: ({self.target_position[0]:.2f}, {self.target_position[1]:.2f}, {self.target_position[2]:.2f})")
            print(f"🎯 Distance: {np.linalg.norm(to_target):.2f}")
        
        # Calculate drone forward direction (corrected coordinate system)
        drone_forward = np.array([math.sin(drone_state.yaw), math.cos(drone_state.yaw), 0.0])
        forward_velocity = np.dot(to_target, drone_forward)
        
        if self.debug_enabled:
            print(f"🎯 Drone forward: ({drone_forward[0]:.2f}, {drone_forward[1]:.2f}, {drone_forward[2]:.2f})")
            print(f"🎯 Forward velocity: {forward_velocity:.2f}")
        
        input = DroneInput()
        
        # Forward thrust - increase scaling
        input.forward_thrust = np.clip(forward_velocity / 10.0, -1.0, 1.0)
        
        # Vertical thrust - increase scaling
        vertical_diff = to_target[2]
        input.vertical_thrust = np.clip(vertical_diff / 10.0, -0.5, 0.5)
        
        # Yaw control - increase scaling
        target_direction = to_target / (np.linalg.norm(to_target) + 1e-6)
        target_yaw = math.atan2(target_direction[1], target_direction[0])
        
        # Calculate yaw error with proper normalization
        yaw_error = target_yaw - drone_state.yaw
        
        # Debug: show the raw values
        if self.debug_enabled:
            print(f"🎯 Raw target_yaw: {target_yaw:.6f} ({target_yaw * 180 / math.pi:.1f}°)")
            print(f"🎯 Raw drone_state.yaw: {drone_state.yaw:.6f} ({drone_state.yaw * 180 / math.pi:.1f}°)")
            print(f"🎯 Raw yaw_error: {yaw_error:.6f} ({yaw_error * 180 / math.pi:.1f}°)")
        
        # Special case: if drone is at ~360° and target is at ~0°, force left turn
        if drone_state.yaw > 5.5 and target_yaw < 0.5:
            yaw_error = -0.5  # Force a left turn
            if self.debug_enabled:
                print(f"🎯 Special case: forcing left turn")
                print(f"🎯 Special case calculation: forcing yaw_error = {yaw_error:.6f}")
        else:
            # Normalize to [-π, π] - this is the key fix
            if yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            elif yaw_error < -math.pi:
                yaw_error += 2 * math.pi
        
        if self.debug_enabled:
            print(f"🎯 Final yaw_error: {yaw_error:.6f} ({yaw_error * 180 / math.pi:.1f}°)")
        
        input.yaw_rate = np.clip(yaw_error * 5.0, -1.0, 1.0)
        
        if self.debug_enabled:
            print(f"🎯 Target direction: ({target_direction[0]:.3f}, {target_direction[1]:.3f}, {target_direction[2]:.3f})")
            print(f"🎯 Target yaw: {target_yaw:.3f} ({target_yaw * 180 / math.pi:.1f}°)")
            print(f"🎯 Current yaw: {drone_state.yaw:.3f} ({drone_state.yaw * 180 / math.pi:.1f}°)")
            print(f"🎯 Yaw error: {yaw_error:.3f} ({yaw_error * 180 / math.pi:.1f}°)")
            print(f"🎯 Input - F:{input.forward_thrust:.3f} Y:{input.yaw_rate:.3f} V:{input.vertical_thrust:.3f}")
        
        return input

def main():
    print("Python Drone AI System")
    print("=" * 30)
    
    # Test the AI
    ai = PathfindingAI()
    drone_state = DroneState(100.0, 200.0, 50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    
    ai.set_mode(AIMode.FOLLOW_PATH)
    ai.set_target(300.0, 300.0, 80.0)
    
    print("\n🧪 Testing AI...")
    ai_input = ai.update(0.016, drone_state)
    
    print(f"\n✅ Test completed!")
    print(f"Forward thrust: {ai_input.forward_thrust:.3f}")
    print(f"Yaw rate: {ai_input.yaw_rate:.3f}")
    print(f"Vertical thrust: {ai_input.vertical_thrust:.3f}")

if __name__ == "__main__":
    main()
