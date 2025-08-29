#include "Camera.h"
#include <cmath>

Camera::Camera() {
    position_ = cv::Point3f(0, 0, 100);
    target_ = cv::Point3f(100, 0, 100);
    up_ = cv::Point3f(0, 0, 1);
    mode_ = CameraMode::FIRST_PERSON;
    
    orbit_angle_x_ = 0.0f;
    orbit_angle_y_ = 0.0f;
    zoom_distance_ = THIRD_PERSON_DISTANCE;
}

void Camera::setFirstPersonMode(const cv::Point3f& drone_pos, const cv::Point3f& drone_orientation) {
    mode_ = CameraMode::FIRST_PERSON;
    
    // Position camera at drone's position
    position_ = drone_pos;
    
    // Calculate forward direction based on drone's yaw
    float yaw = drone_orientation.z;
    float pitch = drone_orientation.y;
    float roll = drone_orientation.x;
    
    // Forward vector (where drone is facing)
    cv::Point3f forward(cos(yaw) * cos(pitch), sin(yaw) * cos(pitch), -sin(pitch));
    
    // Up vector (affected by roll)
    cv::Point3f up(0, 0, 1);
    up = rotateVector(up, cv::Point3f(roll, 0, 0));
    
    // Set camera target in front of drone
    target_ = position_ + forward * 100.0f;
    
    // Set camera up vector
    up_ = up;
}

void Camera::setThirdPersonMode(const cv::Point3f& drone_pos, const cv::Point3f& drone_orientation) {
    mode_ = CameraMode::THIRD_PERSON;
    
    // Calculate camera position behind and above the drone
    float yaw = drone_orientation.z;
    float pitch = drone_orientation.y;
    
    // Camera position behind drone
    float behind_distance = zoom_distance_ * cos(pitch);
    float height_offset = THIRD_PERSON_HEIGHT + zoom_distance_ * sin(pitch);
    
    position_.x = drone_pos.x - cos(yaw) * behind_distance;
    position_.y = drone_pos.y - sin(yaw) * behind_distance;
    position_.z = drone_pos.z + height_offset;
    
    // Look at the drone
    target_ = drone_pos;
    
    // Camera up vector
    up_ = cv::Point3f(0, 0, 1);
    
    // Apply orbit angles for mouse control
    updateThirdPersonPosition(drone_pos, drone_orientation);
}

void Camera::updateThirdPersonPosition(const cv::Point3f& drone_pos, const cv::Point3f& drone_orientation) {
    if (mode_ != CameraMode::THIRD_PERSON) return;
    
    // Calculate orbit position around drone
    cv::Point3f orbit_pos = calculateOrbitPosition(orbit_angle_x_, orbit_angle_y_, zoom_distance_);
    
    // Apply drone's yaw to orbit
    float yaw = drone_orientation.z;
    float orbit_x = orbit_pos.x * cos(yaw) - orbit_pos.y * sin(yaw);
    float orbit_y = orbit_pos.x * sin(yaw) + orbit_pos.y * cos(yaw);
    
    position_.x = drone_pos.x + orbit_x;
    position_.y = drone_pos.y + orbit_y;
    position_.z = drone_pos.z + orbit_pos.z + THIRD_PERSON_HEIGHT;
    
    // Look at drone
    target_ = drone_pos;
}

cv::Point3f Camera::calculateOrbitPosition(float angle_x, float angle_y, float distance) {
    // Calculate position on a sphere around the origin
    float x = distance * cos(angle_y) * cos(angle_x);
    float y = distance * cos(angle_y) * sin(angle_x);
    float z = distance * sin(angle_y);
    
    return cv::Point3f(x, y, z);
}

void Camera::orbit(float delta_x, float delta_y) {
    orbit_angle_x_ += delta_x * 0.01f;
    orbit_angle_y_ += delta_y * 0.01f;
    
    // Clamp vertical angle
    if (orbit_angle_y_ > M_PI / 3.0f) orbit_angle_y_ = M_PI / 3.0f;
    if (orbit_angle_y_ < -M_PI / 3.0f) orbit_angle_y_ = -M_PI / 3.0f;
    
    // Wrap horizontal angle
    orbit_angle_x_ = std::fmod(orbit_angle_x_, 2.0f * M_PI);
}

void Camera::zoom(float delta) {
    zoom_distance_ += delta * 10.0f;
    if (zoom_distance_ < MIN_ZOOM) zoom_distance_ = MIN_ZOOM;
    if (zoom_distance_ > MAX_ZOOM) zoom_distance_ = MAX_ZOOM;
}

void Camera::resetView() {
    orbit_angle_x_ = 0.0f;
    orbit_angle_y_ = 0.0f;
    zoom_distance_ = THIRD_PERSON_DISTANCE;
}

cv::Point3f Camera::rotateVector(const cv::Point3f& vec, const cv::Point3f& angles) {
    // Simple 3D rotation
    float roll = angles.x;
    float pitch = angles.y;
    float yaw = angles.z;
    
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
