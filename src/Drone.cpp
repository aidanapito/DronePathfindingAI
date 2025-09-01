#include "Drone.h"
#include <cmath>
#include <algorithm>

Drone::Drone() {
    state_.x = 100.0f;
    state_.y = 100.0f;
    state_.z = 150.0f;
    state_.vx = 0.0f;
    state_.vy = 0.0f;
    state_.vz = 0.0f;
    state_.roll = 0.0f;
    state_.pitch = 0.0f;
    state_.yaw = 0.0f;
}

void Drone::update(float delta_time, const DroneInput& input) {
    // Apply input forces
    float forward_force = input.forward_thrust * ACCELERATION;
    float yaw_torque = input.yaw_rate * TURN_RATE;
    float pitch_torque = input.pitch_rate * TURN_RATE;
    
    // Adjust roll direction based on yaw orientation to maintain consistent roll behavior
    // When facing south (yaw around 180°), invert the roll direction
    float roll_torque = input.roll_rate * TURN_RATE;
    if (state_.yaw > M_PI && state_.yaw < 2*M_PI) {
        roll_torque = -roll_torque; // Invert roll when facing south (180-360 degrees)
    }
    
    float vertical_force = input.vertical_thrust * ACCELERATION;
    
    // Update orientation
    state_.roll += roll_torque * delta_time;
    state_.pitch += pitch_torque * delta_time;
    state_.yaw += yaw_torque * delta_time;
    
    // Apply orientation damping to prevent extreme values (only when no input is applied)
    if (std::abs(input.roll_rate) < 0.01f) {
        state_.roll *= 0.95f;  // 5% damping per frame when no roll input
    }
    if (std::abs(input.pitch_rate) < 0.01f) {
        state_.pitch *= 0.95f; // 5% damping per frame when no pitch input
    }
    
    // Clamp roll and pitch to prevent flipping
    state_.roll = std::max(-static_cast<float>(M_PI)/4.0f, std::min(static_cast<float>(M_PI)/4.0f, state_.roll));
    state_.pitch = std::max(-static_cast<float>(M_PI)/6.0f, std::min(static_cast<float>(M_PI)/6.0f, state_.pitch));
    
    // Keep yaw in range [0, 2π)
    state_.yaw = std::fmod(state_.yaw + 2.0f*static_cast<float>(M_PI), 2.0f*static_cast<float>(M_PI));
    
    // Calculate forward direction based on yaw and pitch
    // Align with visual orientation: Y is forward, X is right
    float forward_dir_x = sin(state_.yaw) * cos(state_.pitch);  // Right/left component
    float forward_dir_y = cos(state_.yaw) * cos(state_.pitch);  // Forward/backward component
    float forward_dir_z = sin(state_.pitch);
    
    // Apply forces
    state_.vx += forward_dir_x * forward_force * delta_time;
    state_.vy += forward_dir_y * forward_force * delta_time;
    state_.vz += vertical_force * delta_time;
    
    // Apply physics
    applyPhysics(delta_time);
    
    // Constrain movement
    constrainMovement();
}

DroneState Drone::getPosition() const {
    return state_;
}

DroneState Drone::getOrientation() const {
    return state_;
}

DroneState Drone::getOrientationOnly() const {
    // Return only orientation values, with position and velocity set to 0
    DroneState orientation_only;
    orientation_only.x = 0.0f;
    orientation_only.y = 0.0f;
    orientation_only.z = 0.0f;
    orientation_only.vx = 0.0f;
    orientation_only.vy = 0.0f;
    orientation_only.vz = 0.0f;
    orientation_only.roll = state_.roll;
    orientation_only.pitch = state_.pitch;
    orientation_only.yaw = state_.yaw;
    return orientation_only;
}

void Drone::applyPhysics(float delta_time) {
    // Update position
    state_.x += state_.vx * delta_time;
    state_.y += state_.vy * delta_time;
    state_.z += state_.vz * delta_time;
    
    // Apply drag
    state_.vx *= DRAG;
    state_.vy *= DRAG;
    state_.vz *= DRAG;
    
    // Clamp horizontal speed
    float horizontal_speed = sqrt(state_.vx * state_.vx + state_.vy * state_.vy);
    if (horizontal_speed > MAX_SPEED) {
        float scale = MAX_SPEED / horizontal_speed;
        state_.vx *= scale;
        state_.vy *= scale;
    }
}

void Drone::constrainMovement() {
    // Constrain altitude
    if (state_.z < MIN_ALTITUDE) state_.z = MIN_ALTITUDE;
    if (state_.z > MAX_ALTITUDE) state_.z = MAX_ALTITUDE;
}
