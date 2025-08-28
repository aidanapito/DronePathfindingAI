#include "Camera.h"
#include <cmath>

Camera::Camera() {
    position_ = cv::Point3f(0, 0, 100);
    target_ = cv::Point3f(1, 0, 0);
    up_ = cv::Point3f(0, 0, 1);
    mode_ = CameraMode::FIRST_PERSON;
    
    // Third-person camera state
    orbit_angle_x_ = 0.0f;
    orbit_angle_y_ = 0.0f;
    zoom_distance_ = THIRD_PERSON_DISTANCE;
}

void Camera::setFirstPersonMode(const cv::Point3f& drone_pos, const cv::Point3f& drone_orientation) {
    mode_ = CameraMode::FIRST_PERSON;
    
    // Camera is positioned exactly at drone position
    position_ = drone_pos;
    
    // Camera looks in the direction the drone is facing, but much further away for proper perspective
    float yaw = drone_orientation.z;
    target_ = drone_pos + cv::Point3f(cos(yaw) * 100.0f, sin(yaw) * 100.0f, 0); // Look 100 units ahead
    
    // Up vector is always pointing up
    up_ = cv::Point3f(0, 0, 1);
}

void Camera::setThirdPersonMode(const cv::Point3f& drone_pos, const cv::Point3f& drone_orientation) {
    mode_ = CameraMode::THIRD_PERSON;
    
    // Update third-person camera position based on drone
    updateThirdPersonPosition(drone_pos, drone_orientation);
}

void Camera::updateThirdPersonPosition(const cv::Point3f& drone_pos, const cv::Point3f& drone_orientation) {
    // Calculate camera position behind and above the drone
    cv::Point3f offset = calculateOrbitPosition(orbit_angle_x_, orbit_angle_y_, zoom_distance_);
    
    // Apply offset to drone position
    position_ = drone_pos + offset;
    
    // Camera always looks at the drone
    target_ = drone_pos;
    
    // Up vector is always pointing up
    up_ = cv::Point3f(0, 0, 1);
}

cv::Point3f Camera::calculateOrbitPosition(float angle_x, float angle_y, float distance) {
    // Calculate offset based on orbit angles
    float x_offset = -distance * cos(angle_y) * sin(angle_x);
    float y_offset = -distance * cos(angle_y) * cos(angle_x);
    float z_offset = THIRD_PERSON_HEIGHT + distance * sin(angle_y);
    
    return cv::Point3f(x_offset, y_offset, z_offset);
}

void Camera::orbit(float delta_x, float delta_y) {
    if (mode_ == CameraMode::THIRD_PERSON) {
        // Convert mouse movement to orbit angles
        float sensitivity = 0.01f;
        orbit_angle_x_ += delta_x * sensitivity;
        orbit_angle_y_ += delta_y * sensitivity;
        
        // Clamp vertical angle to prevent camera going below ground
        orbit_angle_y_ = std::max(-static_cast<float>(M_PI)/3.0f, std::min(static_cast<float>(M_PI)/3.0f, orbit_angle_y_));
        
        // Wrap horizontal angle
        orbit_angle_x_ = std::fmod(orbit_angle_x_, 2.0f*static_cast<float>(M_PI));
    }
}

void Camera::zoom(float delta) {
    if (mode_ == CameraMode::THIRD_PERSON) {
        // Adjust zoom distance
        zoom_distance_ += delta * 10.0f;
        
        // Clamp zoom distance
        zoom_distance_ = std::clamp(zoom_distance_, MIN_ZOOM, MAX_ZOOM);
    }
}

void Camera::resetView() {
    orbit_angle_x_ = 0.0f;
    orbit_angle_y_ = 0.0f;
    zoom_distance_ = THIRD_PERSON_DISTANCE;
}
