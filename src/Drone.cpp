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
    // Apply input controls
    float forward_thrust = throttle * ACCELERATION;
    float yaw_change = yaw_rate * TURN_RATE * delta_time;
    float pitch_change = pitch_rate * TURN_RATE * delta_time;
    float roll_change = roll_rate * TURN_RATE * delta_time;
    float vertical_change = vertical_thrust * ACCELERATION * delta_time;
    
    // Update orientation
    state_.orientation.x += roll_change;   // Roll
    state_.orientation.y += pitch_change;  // Pitch
    state_.orientation.z += yaw_change;    // Yaw
    
    // Clamp orientation to prevent excessive rotation
    state_.orientation.x = std::max(-static_cast<float>(M_PI)/4.0f, std::min(static_cast<float>(M_PI)/4.0f, state_.orientation.x));  // Max 45° roll
    state_.orientation.y = std::max(-static_cast<float>(M_PI)/6.0f, std::min(static_cast<float>(M_PI)/6.0f, state_.orientation.y));  // Max 30° pitch
    state_.orientation.z = std::fmod(state_.orientation.z, 2.0f*static_cast<float>(M_PI));    // Yaw wraps around
    
    // Calculate forward direction based on yaw
    float forward_x = cos(state_.orientation.z);
    float forward_y = sin(state_.orientation.z);
    
    // Apply forward thrust in drone's facing direction
    acceleration_.x = forward_x * forward_thrust;
    acceleration_.y = forward_y * forward_thrust;
    acceleration_.z = vertical_change;
    
    // Apply physics
    applyPhysics(delta_time);
    
    // Constrain movement
    constrainMovement();
    
    // Update altitude
    state_.altitude = state_.position.z;
}

void Drone::applyPhysics(float delta_time) {
    // Update velocity with acceleration
    state_.velocity += acceleration_ * delta_time;
    
    // Apply air resistance (drag)
    state_.velocity *= DRAG;
    
    // Update position
    state_.position += state_.velocity * delta_time;
    
    // Reset acceleration
    acceleration_ = cv::Point3f(0, 0, 0);
}

void Drone::constrainMovement() {
    // Constrain speed
    float speed = sqrt(state_.velocity.x * state_.velocity.x + 
                      state_.velocity.y * state_.velocity.y + 
                      state_.velocity.z * state_.velocity.z);
    
    if (speed > MAX_SPEED) {
        float scale = MAX_SPEED / speed;
        state_.velocity *= scale;
    }
    
    // Constrain altitude
    if (state_.position.z > MAX_ALTITUDE) {
        state_.position.z = MAX_ALTITUDE;
        if (state_.velocity.z > 0) state_.velocity.z = 0;
    }
    
    if (state_.position.z < MIN_ALTITUDE) {
        state_.position.z = MIN_ALTITUDE;
        if (state_.velocity.z < 0) state_.velocity.z = 0;
    }
}

cv::Point3f Drone::rotateVector(const cv::Point3f& vec, const cv::Point3f& angles) {
    // Simple rotation - in a full implementation this would use rotation matrices
    // For now, we'll just return the vector as-is since we're mainly using yaw
    return vec;
}
