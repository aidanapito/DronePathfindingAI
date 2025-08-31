#include "Camera.h"
#include <cmath>
#include <algorithm>
#include <iostream>

Camera::Camera() {
    position_.x = 0.0f;
    position_.y = 0.0f;
    position_.z = 100.0f;
    target_.x = 100.0f;
    target_.y = 0.0f;
    target_.z = 100.0f;
    up_.x = 0.0f;
    up_.y = 0.0f;
    up_.z = 1.0f;
    
    mode_ = CameraMode::FIRST_PERSON;
    orbit_angle_x_ = 0.0f;
    orbit_angle_y_ = 0.0f;
    zoom_distance_ = 100.0f;
}

void Camera::setFirstPersonMode(const struct DroneState& drone_pos, const struct DroneState& drone_orient) {
    // Set camera position in front of the drone, not inside it
    float yaw = drone_orient.yaw;
    float pitch = drone_orient.pitch;
    float roll = drone_orient.roll;
    
    // Calculate forward direction based on drone orientation
    // In this coordinate system: X=left/right, Y=forward/backward, Z=up/down
    // This must match the drone's coordinate system in Drone.cpp
    // When yaw=0, drone faces forward (+Y direction)
    // When yaw=π/2, drone faces right (+X direction)
    // When yaw=-π/2, drone faces left (-X direction)
    // Note: The rendering uses transformed coordinates (X=left/right, Y=up/down, Z=forward/backward)
    // So we need to transform the forward direction to match
    float forward_x = sin(yaw) * cos(pitch);   // Right/left component (matches drone)
    float forward_y = sin(pitch);              // Up/down component (transformed for rendering)
    float forward_z = cos(yaw) * cos(pitch);   // Forward/backward component (transformed for rendering)
    
    // Position camera 2 units in front of the drone center, slightly elevated
    // The drone's visual front is in the +Y direction, so position camera accordingly
    position_.x = drone_pos.x + forward_x * 2.0f;
    position_.y = drone_pos.y + forward_y * 2.0f;
    position_.z = drone_pos.z + forward_z * 2.0f + 0.5f;  // Move 0.5 units up from calculated position
    
    // Debug output to understand coordinate system
    std::cout << "Setting 1st person: Drone pos(" << drone_pos.x << ", " << drone_pos.y << ", " << drone_pos.z 
              << ") yaw:" << yaw << " | Forward dir(" << forward_x << ", " << forward_y << ", " << forward_z 
              << ") | Camera pos(" << position_.x << ", " << position_.y << ", " << position_.z << ")" << std::endl;
    
    // Calculate up vector
    float up_x = 0.0f;
    float up_y = 0.0f;
    float up_z = 1.0f;
    
    // Apply roll to up vector with reduced sensitivity for more stable camera
    CameraState roll_angles = {roll * CAMERA_ROLL_SENSITIVITY, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    CameraState rotated_up = rotateVector({up_x, up_y, up_z, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, roll_angles);
    
    // Set target 100 units ahead in the direction the drone is facing
    target_.x = position_.x + forward_x * 100.0f;
    target_.y = position_.y + forward_y * 100.0f;
    target_.z = position_.z + forward_z * 100.0f;
    
    // Set up vector - use rotated up for roll effect, but ensure it's normalized
    up_.x = rotated_up.x;
    up_.y = rotated_up.y;
    up_.z = rotated_up.z;
    
    // Normalize up vector to prevent issues
    float up_length = sqrt(up_.x * up_.x + up_.y * up_.y + up_.z * up_.z);
    if (up_length > 0.001f) {
        up_.x /= up_length;
        up_.y /= up_length;
        up_.z /= up_length;
    } else {
        // Fallback to world up if rotation failed
        up_.x = 0.0f;
        up_.y = 0.0f;
        up_.z = 1.0f;
    }
    
    mode_ = CameraMode::FIRST_PERSON;
}

void Camera::setThirdPersonMode(const struct DroneState& drone_pos, const struct DroneState& drone_orient) {
    // In third-person mode, the red dot stays fixed in the center of the screen
    // The camera moves around the red dot, which represents the drone's position
    
    // Debug output to understand coordinate system
    std::cout << "Setting 3rd person: Drone pos(" << drone_pos.x << ", " << drone_pos.y << ", " << drone_pos.z 
              << ") yaw:" << drone_orient.yaw << " | Camera pos(" << drone_pos.x << ", " << (drone_pos.y - 100.0f) << ", " << (drone_pos.z + THIRD_PERSON_HEIGHT) << ")" << std::endl;
    
    // Position camera behind and above the drone's position
    float offset_x = 0.0f;                    // No left/right offset - camera stays centered
    float offset_y = THIRD_PERSON_HEIGHT;     // Above the drone (Y is now up/down)
    float offset_z = -100.0f;                 // Behind the drone (Z is now forward/backward)
    
    position_.x = drone_pos.x + offset_x;
    position_.y = drone_pos.y + offset_y;
    position_.z = drone_pos.z + offset_z;
    
    // Always look at the drone's position (where the red dot will be)
    target_.x = drone_pos.x;
    target_.y = drone_pos.y;
    target_.z = drone_pos.z;
    
    // Set up vector - always point up in world space
    up_.x = 0.0f;
    up_.y = 0.0f;
    up_.z = 1.0f;
    
    mode_ = CameraMode::THIRD_PERSON;
}

void Camera::switchMode(const struct DroneState& drone_pos, const struct DroneState& drone_orient) {
    if (mode_ == CameraMode::FIRST_PERSON) {
        setThirdPersonMode(drone_pos, drone_orient);
    } else {
        setFirstPersonMode(drone_pos, drone_orient);
    }
}

void Camera::updateFirstPersonPosition(const struct DroneState& drone_pos, const struct DroneState& drone_orient) {
    // Set camera position in front of the drone, not inside it
    float yaw = drone_orient.yaw;
    float pitch = drone_orient.pitch;
    float roll = drone_orient.roll;
    
    // Calculate forward direction based on drone orientation
    // In this coordinate system: X=left/right, Y=forward/backward, Z=up/down
    // This must match the drone's coordinate system in Drone.cpp
    float forward_x = sin(yaw) * cos(pitch);   // Right/left component (matches drone)
    float forward_y = cos(yaw) * cos(pitch);   // Forward/backward component (matches drone)
    float forward_z = sin(pitch);              // Up/down component (matches drone)
    
    // Position camera 2 units in front of the drone center, slightly elevated
    position_.x = drone_pos.x + forward_x * 2.0f;
    position_.y = drone_pos.y + forward_y * 2.0f;
    position_.z = drone_pos.z + forward_z * 2.0f + 0.5f;  // Move 0.5 units up from calculated position
    
    // Calculate up vector
    float up_x = 0.0f;
    float up_y = 0.0f;
    float up_z = 1.0f;
    
    // Apply roll to up vector with reduced sensitivity for more stable camera
    CameraState roll_angles = {roll * CAMERA_ROLL_SENSITIVITY, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    CameraState rotated_up = rotateVector({up_x, up_y, up_z, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, roll_angles);
    
    // Set target 100 units ahead in the direction the drone is facing
    target_.x = position_.x + forward_x * 100.0f;
    target_.y = position_.y + forward_y * 100.0f;
    target_.z = position_.z + forward_z * 100.0f;
    
    // Set up vector - use rotated up for roll effect, but ensure it's normalized
    up_.x = rotated_up.x;
    up_.y = rotated_up.y;
    up_.z = rotated_up.z;
    
    // Normalize up vector to prevent issues
    float up_length = sqrt(up_.x * up_.x + up_.y * up_.y + up_.z * up_.z);
    if (up_length > 0.001f) {
        up_.x /= up_length;
        up_.y /= up_length;
        up_.z /= up_length;
    } else {
        // Fallback to world up if rotation failed
        up_.x = 0.0f;
        up_.y = 0.0f;
        up_.z = 1.0f;
    }
}

void Camera::update(const struct DroneState& drone_pos, const struct DroneState& drone_orient) {
    if (mode_ == CameraMode::FIRST_PERSON) {
        updateFirstPersonPosition(drone_pos, drone_orient);
    } else {
        updateThirdPersonPosition(drone_pos, drone_orient);
    }
}

void Camera::updateThirdPersonPosition(const struct DroneState& drone_pos, const struct DroneState& drone_orient) {
    // Position camera behind and above the drone, following its forward direction
    float yaw = drone_orient.yaw;
    
    // Calculate camera position behind the drone (in the direction it's facing)
    // When drone faces forward (yaw = 0), camera should be behind (negative Y)
    // When drone faces right (yaw = π/2), camera should be to the left (negative X)
    float offset_x = -sin(yaw) * 100.0f;     // Left/right offset based on yaw
    float offset_y = -cos(yaw) * 100.0f;     // Forward/backward offset based on yaw
    float offset_z = THIRD_PERSON_HEIGHT;     // Above the drone
    
    position_.x = drone_pos.x + offset_x;
    position_.y = drone_pos.y + offset_y;
    position_.z = drone_pos.z + offset_z;
    
    // Look at drone
    target_.x = drone_pos.x;
    target_.y = drone_pos.y;
    target_.z = drone_pos.z;
}

CameraState Camera::calculateOrbitPosition(float angle_x, float angle_y, float distance) {
    // Calculate position on a sphere
    // Adjust coordinate system so camera starts behind the drone (negative Y)
    float x = distance * cos(angle_y) * sin(angle_x);
    float y = -distance * cos(angle_y) * cos(angle_x); // Negative to start behind
    float z = distance * sin(angle_y);
    
    return {x, y, z, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
}

void Camera::orbit(float delta_x, float delta_y) {
    orbit_angle_x_ += delta_x * ORBIT_SENSITIVITY;
    orbit_angle_y_ += delta_y * ORBIT_SENSITIVITY;
    
    // Clamp vertical angle
    orbit_angle_y_ = std::max(-static_cast<float>(M_PI)/3.0f, std::min(static_cast<float>(M_PI)/3.0f, orbit_angle_y_));
    
    // Keep horizontal angle in range
    orbit_angle_x_ = std::fmod(orbit_angle_x_, 2.0f*static_cast<float>(M_PI));
}

void Camera::handleMouseInput(float delta_x, float delta_y) {
    if (mode_ == CameraMode::FIRST_PERSON) {
        // In first person mode, mouse controls the camera's look direction
        // This would typically control pitch and yaw of the view
        // For now, we'll just orbit around the current position
        orbit(delta_x, delta_y);
    } else {
        // In third person mode, mouse controls the camera orbit around the drone
        orbit(delta_x, delta_y);
    }
}

void Camera::zoom(float delta) {
    zoom_distance_ += delta * ZOOM_SENSITIVITY;
    zoom_distance_ = std::max(MIN_ZOOM, std::min(MAX_ZOOM, zoom_distance_));
}

void Camera::resetView() {
    orbit_angle_x_ = 0.0f;        // Start looking straight ahead
    orbit_angle_y_ = 0.0f;        // Start at drone level
    zoom_distance_ = 100.0f;      // Start at medium distance
}

CameraState Camera::rotateVector(const CameraState& vec, const CameraState& angles) {
    // Simple 3D rotation (can be improved with proper rotation matrices)
    float roll = angles.x;
    float pitch = angles.y;
    float yaw = angles.z;
    
    // Apply rotations in order: roll -> pitch -> yaw
    float x = vec.x;
    float y = vec.y;
    float z = vec.z;
    
    // Roll (around X-axis)
    float y_roll = y * cos(roll) - z * sin(roll);
    float z_roll = y * sin(roll) + z * cos(roll);
    y = y_roll;
    z = z_roll;
    
    // Pitch (around Y-axis)
    float x_pitch = x * cos(pitch) + z * sin(pitch);
    float z_pitch = -x * sin(pitch) + z * cos(pitch);
    x = x_pitch;
    z = z_pitch;
    
    // Yaw (around Z-axis)
    float x_yaw = x * cos(yaw) - y * sin(yaw);
    float y_yaw = x * sin(yaw) + y * cos(yaw);
    x = x_yaw;
    y = y_yaw;
    
    return {x, y, z, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
}
