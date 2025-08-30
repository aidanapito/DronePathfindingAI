#include "Camera.h"
#include <cmath>
#include <algorithm>

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
    // Set camera position to drone position
    position_.x = drone_pos.x;
    position_.y = drone_pos.y;
    position_.z = drone_pos.z;
    
    // Calculate forward direction based on drone orientation
    float yaw = drone_orient.yaw;
    float pitch = drone_orient.pitch;
    float roll = drone_orient.roll;
    
    // Calculate forward vector
    float forward_x = cos(yaw) * cos(pitch);
    float forward_y = sin(yaw) * cos(pitch);
    float forward_z = -sin(pitch);
    
    // Calculate up vector
    float up_x = 0.0f;
    float up_y = 0.0f;
    float up_z = 1.0f;
    
    // Apply roll to up vector with reduced sensitivity
    CameraState roll_angles = {roll * CAMERA_ROLL_SENSITIVITY, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    CameraState rotated_up = rotateVector({up_x, up_y, up_z, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, roll_angles);
    
    // Set target 100 units ahead
    target_.x = position_.x + forward_x * 100.0f;
    target_.y = position_.y + forward_y * 100.0f;
    target_.z = position_.z + forward_z * 100.0f;
    
    // Set up vector
    up_.x = rotated_up.x;
    up_.y = rotated_up.y;
    up_.z = rotated_up.z;
    
    mode_ = CameraMode::FIRST_PERSON;
}

void Camera::setThirdPersonMode(const struct DroneState& drone_pos, const struct DroneState& drone_orient) {
    // Calculate camera position behind and above the drone
    float yaw = drone_orient.yaw;
    float pitch = drone_orient.pitch;
    
    // Position camera behind drone
    float offset_x = -cos(yaw) * zoom_distance_;
    float offset_y = -sin(yaw) * zoom_distance_;
    float offset_z = THIRD_PERSON_HEIGHT;
    
    position_.x = drone_pos.x + offset_x;
    position_.y = drone_pos.y + offset_y;
    position_.z = drone_pos.z + offset_z;
    
    // Look at drone
    target_.x = drone_pos.x;
    target_.y = drone_pos.y;
    target_.z = drone_pos.z;
    
    // Set up vector
    up_.x = 0.0f;
    up_.y = 0.0f;
    up_.z = 1.0f;
    
    mode_ = CameraMode::THIRD_PERSON;
}

void Camera::updateFirstPersonPosition(const struct DroneState& drone_pos, const struct DroneState& drone_orient) {
    // Set camera position to drone position
    position_.x = drone_pos.x;
    position_.y = drone_pos.y;
    position_.z = drone_pos.z;
    
    // Calculate forward direction based on drone orientation
    float yaw = drone_orient.yaw;
    float pitch = drone_orient.pitch;
    float roll = drone_orient.roll;
    
    // Calculate forward vector
    float forward_x = cos(yaw) * cos(pitch);
    float forward_y = sin(yaw) * cos(pitch);
    float forward_z = -sin(pitch);
    
    // Calculate up vector
    float up_x = 0.0f;
    float up_y = 0.0f;
    float up_z = 1.0f;
    
    // Apply roll to up vector with reduced sensitivity
    CameraState roll_angles = {roll * CAMERA_ROLL_SENSITIVITY, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    CameraState rotated_up = rotateVector({up_x, up_y, up_z, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, roll_angles);
    
    // Set target 100 units ahead
    target_.x = position_.x + forward_x * 100.0f;
    target_.y = position_.y + forward_y * 100.0f;
    target_.z = position_.z + forward_z * 100.0f;
    
    // Set up vector
    up_.x = rotated_up.x;
    up_.y = rotated_up.y;
    up_.z = rotated_up.z;
}

void Camera::update(const struct DroneState& drone_pos, const struct DroneState& drone_orient) {
    if (mode_ == CameraMode::FIRST_PERSON) {
        updateFirstPersonPosition(drone_pos, drone_orient);
    } else {
        updateThirdPersonPosition(drone_pos, drone_orient);
    }
}

void Camera::updateThirdPersonPosition(const struct DroneState& drone_pos, const struct DroneState& drone_orient) {
    // Calculate orbit position
    CameraState orbit_pos = calculateOrbitPosition(orbit_angle_x_, orbit_angle_y_, zoom_distance_);
    
    // Apply drone's yaw to the orbit
    float yaw = drone_orient.yaw;
    float rotated_x = orbit_pos.x * cos(yaw) - orbit_pos.y * sin(yaw);
    float rotated_y = orbit_pos.x * sin(yaw) + orbit_pos.y * cos(yaw);
    
    // Set camera position
    position_.x = drone_pos.x + rotated_x;
    position_.y = drone_pos.y + rotated_y;
    position_.z = drone_pos.z + orbit_pos.z + THIRD_PERSON_HEIGHT;
    
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
