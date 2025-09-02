#!/usr/bin/env python3
"""
Python AI Controller for C++ Drone Simulation
This module provides a Python-based AI system that can control the C++ drone simulation.
"""

import numpy as np
import json
import time
import math
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass
from enum import Enum

class AIMode(Enum):
    """AI operation modes"""
    MANUAL = 0
    FOLLOW_PATH = 1
    EXPLORE = 2
    RETURN_HOME = 3
    AVOID_OBSTACLES = 4

class AIState(Enum):
    """AI internal states"""
    IDLE = 0
    PLANNING_PATH = 1
    FOLLOWING_PATH = 2
    AVOIDING_OBSTACLE = 3
    COMPLETED_MISSION = 4
    ERROR = 5

@dataclass
class DroneState:
    """Drone state information"""
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
    """Drone control inputs"""
    forward_thrust: float = 0.0
    yaw_rate: float = 0.0
    pitch_rate: float = 0.0
    roll_rate: float = 0.0
    vertical_thrust: float = 0.0

@dataclass
class Obstacle:
    """Obstacle/building information"""
    x: float
    y: float
    z: float
    width: float
    height: float
    radius: float

class PathfindingAI:
    """Python-based AI controller for drone pathfinding"""
    
    def __init__(self):
        self.current_mode = AIMode.MANUAL
        self.current_state = AIState.IDLE
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.current_path: List[np.ndarray] = []
        self.current_waypoint_index = 0
        self.waypoint_reached_threshold = 10.0
        self.max_velocity = 50.0
        self.turning_rate = 2.0
        self.pathfinding_aggressiveness = 0.5
        
        # Debug output
        self.debug_enabled = True
        
    def set_mode(self, mode: AIMode):
        """Set the AI mode"""
        self.current_mode = mode
        if mode == AIMode.FOLLOW_PATH:
            self.clear_path()
            self.current_state = AIState.IDLE
            if self.debug_enabled:
                print(f"🤖 AI Mode set to: {mode.name}")
    
    def set_target(self, x: float, y: float, z: float):
        """Set the target position"""
        self.target_position = np.array([x, y, z])
        if self.debug_enabled:
            print(f"🎯 Target set to: ({x}, {y}, {z})")
    
    def clear_path(self):
        """Clear the current path"""
        self.current_path = []
        self.current_waypoint_index = 0
    
    def update(self, delta_time: float, drone_state: DroneState, obstacles: List[Obstacle]) -> DroneInput:
        """Main AI update function - returns drone control inputs"""
        
        if self.current_mode == AIMode.MANUAL:
            return DroneInput()
        
        elif self.current_mode == AIMode.FOLLOW_PATH:
            return self._generate_path_following_input(drone_state, obstacles, delta_time)
        
        elif self.current_mode == AIMode.AVOID_OBSTACLES:
            return self._generate_obstacle_avoidance_input(drone_state, obstacles, delta_time)
        
        return DroneInput()
    
    def _generate_path_following_input(self, drone_state: DroneState, obstacles: List[Obstacle], delta_time: float) -> DroneInput:
        """Generate inputs for path following mode"""
        
        # Check if we need to plan a path
        if self.current_state in [AIState.IDLE, AIState.PLANNING_PATH]:
            self._update_path(drone_state, obstacles)
        
        # If we have an error or no path after planning, try to replan
        if not self.current_path:
            if self.current_state == AIState.ERROR:
                if self.debug_enabled:
                    print("🔄 Path planning failed, attempting to replan...")
                self.current_state = AIState.IDLE
                return DroneInput()
            else:
                if self.debug_enabled:
                    print("⚠️  No path available, attempting direct movement")
                return self._generate_direct_movement_input(drone_state)
        
        # Check if we've successfully landed on the target
        if self._has_landed_on_target(drone_state, obstacles):
            self.current_state = AIState.COMPLETED_MISSION
            if self.debug_enabled:
                print("🎉 SUCCESS! Drone has landed on the target building!")
            return DroneInput()
        
        # Get current waypoint
        if self.current_waypoint_index >= len(self.current_path):
            self.current_state = AIState.COMPLETED_MISSION
            return DroneInput()
        
        current_waypoint = self.current_path[self.current_waypoint_index]
        
        # Check if we've reached the current waypoint
        if self._is_waypoint_reached(drone_state, current_waypoint):
            self._advance_to_next_waypoint()
            if self.current_waypoint_index >= len(self.current_path):
                self.current_state = AIState.COMPLETED_MISSION
                return DroneInput()
            current_waypoint = self.current_path[self.current_waypoint_index]
        
        # Calculate desired velocity towards waypoint
        desired_velocity = self._calculate_desired_velocity(drone_state, current_waypoint)
        
        # Convert velocity to drone input
        input = DroneInput()
        
        # Calculate the desired direction relative to drone's current orientation
        drone_pos = np.array([drone_state.x, drone_state.y, drone_state.z])
        to_waypoint = current_waypoint - drone_pos
        
        if self.debug_enabled:
            print(f"🎯 Waypoint following:")
            print(f"   Drone pos: ({drone_pos[0]:.2f}, {drone_pos[1]:.2f}, {drone_pos[2]:.2f})")
            print(f"   Waypoint: ({current_waypoint[0]:.2f}, {current_waypoint[1]:.2f}, {current_waypoint[2]:.2f})")
            print(f"   To waypoint: ({to_waypoint[0]:.2f}, {to_waypoint[1]:.2f}, {to_waypoint[2]:.2f})")
        
        # Project the desired direction onto the drone's forward direction
        # Using the correct coordinate system: (sin(yaw), cos(yaw)) for forward direction
        drone_forward = np.array([math.sin(drone_state.yaw), math.cos(drone_state.yaw), 0.0])
        forward_velocity = np.dot(to_waypoint, drone_forward)
        
        if self.debug_enabled:
            print(f"   Drone yaw: {drone_state.yaw:.2f} radians ({drone_state.yaw * 180.0 / math.pi:.1f} degrees)")
            print(f"   Drone forward: ({drone_forward[0]:.2f}, {drone_forward[1]:.2f}, {drone_forward[2]:.2f})")
            print(f"   Forward velocity: {forward_velocity:.2f}")
        
        input.forward_thrust = np.clip(forward_velocity / self.max_velocity, -1.0, 1.0)
        
        # Vertical thrust based on desired vertical velocity
        vertical_velocity = desired_velocity[2]  # Z is up in our coordinate system
        input.vertical_thrust = np.clip(vertical_velocity / self.max_velocity, -1.0, 1.0)
        
        # Yaw control to face the waypoint
        current_direction = np.array([math.sin(drone_state.yaw), math.cos(drone_state.yaw), 0.0])
        target_direction = np.array([current_waypoint[0] - drone_state.x, 
                                   current_waypoint[1] - drone_state.y, 0.0])
        target_direction = target_direction / (np.linalg.norm(target_direction) + 1e-6)
        
        # Calculate yaw error (target yaw - current yaw)
        target_yaw = math.atan2(target_direction[1], target_direction[0])
        yaw_error = target_yaw - drone_state.yaw
        
        # Normalize yaw error to [-π, π]
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi
        
        input.yaw_rate = np.clip(yaw_error * self.turning_rate, -1.0, 1.0)
        
        # Pitch control for vertical movement
        horizontal_velocity = math.sqrt(desired_velocity[0]**2 + desired_velocity[1]**2)
        pitch_error = math.atan2(desired_velocity[2], horizontal_velocity + 1e-6)
        input.pitch_rate = np.clip(pitch_error * self.turning_rate, -1.0, 1.0)
        
        if self.debug_enabled:
            print(f"   Final input - F:{input.forward_thrust:.3f} Y:{input.yaw_rate:.3f} P:{input.pitch_rate:.3f} V:{input.vertical_thrust:.3f}")
        
        return input
    
    def _generate_direct_movement_input(self, drone_state: DroneState) -> DroneInput:
        """Generate direct movement input when path planning fails"""
        drone_pos = np.array([drone_state.x, drone_state.y, drone_state.z])
        to_target = self.target_position - drone_pos
        
        # Calculate distance to target
        distance_to_target = np.linalg.norm(to_target)
        
        if self.debug_enabled:
            print(f"   Distance to target: {distance_to_target:.2f} units")
            print(f"   Target position: ({self.target_position[0]:.2f}, {self.target_position[1]:.2f}, {self.target_position[2]:.2f})")
            print(f"   Drone position: ({drone_pos[0]:.2f}, {drone_pos[1]:.2f}, {drone_pos[2]:.2f})")
            print(f"   Direction vector: ({to_target[0]:.2f}, {to_target[1]:.2f}, {to_target[2]:.2f})")
        
        # If we're very close to target, just hover
        if distance_to_target < 10.0:
            if self.debug_enabled:
                print("   Very close to target, hovering...")
            return DroneInput()
        
        # Project onto drone's forward direction
        drone_forward = np.array([math.sin(drone_state.yaw), math.cos(drone_state.yaw), 0.0])
        forward_velocity = np.dot(to_target, drone_forward)
        
        if self.debug_enabled:
            print(f"   Drone forward: ({drone_forward[0]:.2f}, {drone_forward[1]:.2f}, {drone_forward[2]:.2f})")
            print(f"   Forward velocity: {forward_velocity:.2f}")
        
        input = DroneInput()
        
        # Limit forward thrust to prevent spiraling
        input.forward_thrust = np.clip(forward_velocity / self.max_velocity, -0.5, 0.5)
        
        # More conservative vertical movement
        vertical_diff = to_target[2]
        input.vertical_thrust = np.clip(vertical_diff / self.max_velocity, -0.3, 0.3)
        
        # Yaw control
        target_direction = to_target / (np.linalg.norm(to_target) + 1e-6)
        target_yaw = math.atan2(target_direction[1], target_direction[0])
        yaw_error = target_yaw - drone_state.yaw
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi
        
        input.yaw_rate = np.clip(yaw_error * self.turning_rate, -0.5, 0.5)
        
        if self.debug_enabled:
            print(f"   Input - F:{input.forward_thrust:.3f} Y:{input.yaw_rate:.3f} V:{input.vertical_thrust:.3f}")
        
        return input
    
    def _update_path(self, drone_state: DroneState, obstacles: List[Obstacle]):
        """Update the path to the target"""
        if self.current_state == AIState.IDLE:
            self.current_state = AIState.PLANNING_PATH
        
        if self.current_state != AIState.PLANNING_PATH:
            return
        
        if self.debug_enabled:
            print("🛣️  Planning path to target...")
            print(f"   Drone at: ({drone_state.x:.2f}, {drone_state.y:.2f}, {drone_state.z:.2f})")
            print(f"   Target at: ({self.target_position[0]:.2f}, {self.target_position[1]:.2f}, {self.target_position[2]:.2f})")
        
        # Check if target is reasonable
        distance_to_target = np.linalg.norm(self.target_position - np.array([drone_state.x, drone_state.y, drone_state.z]))
        
        if self.debug_enabled:
            print(f"   Distance to target: {distance_to_target:.2f} units")
        
        if distance_to_target < 1.0:
            if self.debug_enabled:
                print("   Target is very close, no path planning needed")
            self.current_state = AIState.COMPLETED_MISSION
            return
        
        # Simple direct path for now (can be enhanced with A* later)
        start = np.array([drone_state.x, drone_state.y, drone_state.z])
        
        # Check if we have line of sight to target
        if self._has_line_of_sight(start, self.target_position, obstacles):
            self.current_path = [start, self.target_position]
            self.current_state = AIState.FOLLOWING_PATH
            self.current_waypoint_index = 0
            
            if self.debug_enabled:
                print("✅ Direct line of sight found!")
                print(f"✅ Path planned with {len(self.current_path)} waypoints")
                for i, waypoint in enumerate(self.current_path):
                    print(f"   Waypoint {i}: ({waypoint[0]:.2f}, {waypoint[1]:.2f}, {waypoint[2]:.2f})")
        else:
            # For now, just use direct path even if there are obstacles
            # This can be enhanced with proper A* pathfinding later
            self.current_path = [start, self.target_position]
            self.current_state = AIState.FOLLOWING_PATH
            self.current_waypoint_index = 0
            
            if self.debug_enabled:
                print("🔄 No direct line of sight, using direct path (obstacles ignored for now)")
                print(f"✅ Path planned with {len(self.current_path)} waypoints")
    
    def _has_line_of_sight(self, start: np.ndarray, end: np.ndarray, obstacles: List[Obstacle]) -> bool:
        """Check if there's a clear line of sight between two points"""
        # Simple implementation - can be enhanced with proper collision detection
        return True  # For now, assume we always have line of sight
    
    def _is_waypoint_reached(self, drone_state: DroneState, waypoint: np.ndarray) -> bool:
        """Check if the drone has reached the current waypoint"""
        current_pos = np.array([drone_state.x, drone_state.y, drone_state.z])
        distance = np.linalg.norm(waypoint - current_pos)
        return distance < self.waypoint_reached_threshold
    
    def _advance_to_next_waypoint(self):
        """Advance to the next waypoint"""
        self.current_waypoint_index += 1
        if self.current_waypoint_index < len(self.current_path):
            if self.debug_enabled:
                next_waypoint = self.current_path[self.current_waypoint_index]
                print(f"🎯 Reached waypoint {self.current_waypoint_index - 1}, "
                      f"moving to waypoint {self.current_waypoint_index} "
                      f"at ({next_waypoint[0]:.2f}, {next_waypoint[1]:.2f}, {next_waypoint[2]:.2f})")
        else:
            if self.debug_enabled:
                print("🎯 Reached final waypoint, mission complete!")
    
    def _calculate_desired_velocity(self, drone_state: DroneState, target: np.ndarray) -> np.ndarray:
        """Calculate desired velocity towards target"""
        current_pos = np.array([drone_state.x, drone_state.y, drone_state.z])
        direction = target - current_pos
        distance = np.linalg.norm(direction)
        
        if distance > 0:
            direction = direction / distance
        
        # Scale velocity based on distance and aggressiveness
        speed = min(distance * self.pathfinding_aggressiveness, self.max_velocity)
        return direction * speed
    
    def _has_landed_on_target(self, drone_state: DroneState, obstacles: List[Obstacle]) -> bool:
        """Check if the drone has landed on the target building"""
        # Find the target building (green building)
        target_building = None
        for obstacle in obstacles:
            # For now, assume the target is the first building we find
            # This can be enhanced with proper target identification
            target_building = obstacle
            break
        
        if not target_building:
            return False
        
        # Check if drone is on top of the target building
        drone_x, drone_y, drone_z = drone_state.x, drone_state.y, drone_state.z
        building_x, building_y = target_building.x, target_building.y
        building_top_z = target_building.z + target_building.height
        
        # Check horizontal distance (within building radius)
        horizontal_distance = math.sqrt((drone_x - building_x)**2 + (drone_y - building_y)**2)
        
        # Check vertical distance (close to building top)
        vertical_distance = abs(drone_z - building_top_z)
        
        # Check velocity (moving slowly)
        velocity = math.sqrt(drone_state.vx**2 + drone_state.vy**2 + drone_state.vz**2)
        
        return (horizontal_distance <= target_building.radius and 
                vertical_distance <= 5.0 and 
                velocity < 5.0)
    
    def _generate_obstacle_avoidance_input(self, drone_state: DroneState, obstacles: List[Obstacle], delta_time: float) -> DroneInput:
        """Generate inputs for obstacle avoidance mode"""
        # Simple obstacle avoidance - can be enhanced
        input = DroneInput()
        input.forward_thrust = 0.3  # Slow forward movement
        return input

def main():
    """Test the Python AI system"""
    print("Python Drone AI System")
    print("This module provides AI functionality for the C++ drone simulation")
    print("To use this AI with the C++ simulation, you'll need to:")
    print("1. Modify the C++ simulation to communicate with this Python AI")
    print("2. Use a communication method like JSON files, pipes, or network sockets")
    print("3. Convert between C++ and Python data structures")

if __name__ == "__main__":
    main()
