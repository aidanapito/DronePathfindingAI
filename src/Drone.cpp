#include "Drone.h"
#include <cmath>
#include <algorithm>

Drone::Drone(const cv::Point3f& start_pos) {
    state_.position = start_pos;
    state_.velocity = cv::Point3f(0, 0, 0);
    state_.orientation = cv::Point3f(0, 0, 0); // roll, pitch, yaw
    state_.altitude = start_pos.z;
    
    acceleration_ = cv::Point3f(0, 0, 0);
}

void Drone::update(float delta_time, float throttle, float yaw_rate, 
                   float pitch_rate, float roll_rate, float vertical_thrust) {
    // Apply input forces
    float forward_force = throttle * ACCELERATION;
    float yaw_torque = yaw_rate * TURN_RATE;
    float pitch_torque = pitch_rate * TURN_RATE;
    float roll_torque = roll_rate * TURN_RATE;
    float vertical_force = vertical_thrust * ACCELERATION;
    
    // Update orientation (Euler angles)
    state_.orientation.x += roll_torque * delta_time;   // Roll
    state_.orientation.y += pitch_torque * delta_time;  // Pitch  
    state_.orientation.z += yaw_torque * delta_time;    // Yaw
    
    // Clamp orientation angles
    state_.orientation.x = std::max(-static_cast<float>(M_PI)/4.0f, std::min(static_cast<float>(M_PI)/4.0f, state_.orientation.x));
    state_.orientation.y = std::max(-static_cast<float>(M_PI)/4.0f, std::min(static_cast<float>(M_PI)/4.0f, state_.orientation.y));
    state_.orientation.z = std::fmod(state_.orientation.z, 2.0f*static_cast<float>(M_PI));
    
    // Calculate forward direction based on yaw
    float yaw = state_.orientation.z;
    cv::Point3f forward_dir(cos(yaw), sin(yaw), 0);
    
    // Apply forward force in the direction the drone is facing
    acceleration_.x = forward_dir.x * forward_force;
    acceleration_.y = forward_dir.y * forward_force;
    acceleration_.z = vertical_force;
    
    // Apply physics
    applyPhysics(delta_time);
    
    // Constrain movement
    constrainMovement();
    
    // Update altitude
    state_.altitude = state_.position.z;
}

void Drone::applyPhysics(float delta_time) {
    // Update velocity with acceleration
    state_.velocity.x += acceleration_.x * delta_time;
    state_.velocity.y += acceleration_.y * delta_time;
    state_.velocity.z += acceleration_.z * delta_time;
    
    // Apply drag
    state_.velocity.x *= DRAG;
    state_.velocity.y *= DRAG;
    state_.velocity.z *= DRAG;
    
    // Clamp velocity
    float horizontal_speed = sqrt(state_.velocity.x * state_.velocity.x + state_.velocity.y * state_.velocity.y);
    if (horizontal_speed > MAX_SPEED) {
        float scale = MAX_SPEED / horizontal_speed;
        state_.velocity.x *= scale;
        state_.velocity.y *= scale;
    }
    
    // Update position
    state_.position.x += state_.velocity.x * delta_time;
    state_.position.y += state_.velocity.y * delta_time;
    state_.position.z += state_.velocity.z * delta_time;
}

void Drone::constrainMovement() {
    // Keep drone above ground
    if (state_.position.z < MIN_ALTITUDE) {
        state_.position.z = MIN_ALTITUDE;
        state_.velocity.z = 0;
    }
    
    // Keep drone below max altitude
    if (state_.position.z > MAX_ALTITUDE) {
        state_.position.z = MAX_ALTITUDE;
        state_.velocity.z = 0;
    }
}

cv::Point3f Drone::rotateVector(const cv::Point3f& vec, const cv::Point3f& angles) {
    // Simple 3D rotation (can be improved with proper rotation matrices)
    float roll = angles.x;
    float pitch = angles.y;
    float yaw = angles.z;
    
    // Apply rotations in order: roll -> pitch -> yaw
    cv::Point3f rotated = vec;
    
    // Roll (around X-axis)
    float y_roll = rotated.y * cos(roll) - rotated.z * sin(roll);
    float z_roll = rotated.y * sin(roll) + rotated.z * cos(roll);
    rotated.y = y_roll;
    rotated.z = z_roll;
    
    // Pitch (around Y-axis)
    float x_pitch = rotated.x * cos(pitch) + rotated.z * sin(pitch);
    float z_pitch = -rotated.x * sin(pitch) + rotated.z * cos(pitch);
    rotated.x = x_pitch;
    rotated.z = z_pitch;
    
    // Yaw (around Z-axis)
    float x_yaw = rotated.x * cos(yaw) - rotated.y * sin(yaw);
    float y_yaw = rotated.x * sin(yaw) + rotated.y * cos(yaw);
    rotated.x = x_yaw;
    rotated.y = y_yaw;
    
    return rotated;
}
