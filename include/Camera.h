#pragma once

#include <opencv2/opencv.hpp>

enum class CameraMode {
    FIRST_PERSON,   // Camera is the drone (you see through drone's eyes)
    THIRD_PERSON    // Camera follows behind the drone
};

class Camera {
public:
    Camera();
    
    // Camera positioning
    void setFirstPersonMode(const cv::Point3f& drone_pos, const cv::Point3f& drone_orientation);
    void setThirdPersonMode(const cv::Point3f& drone_pos, const cv::Point3f& drone_orientation);
    
    // Camera controls
    void orbit(float delta_x, float delta_y);  // Mouse movement for third-person
    void zoom(float delta);                    // Mouse wheel
    void resetView();
    
    // Helper methods
    cv::Point3f rotateVector(const cv::Point3f& vec, const cv::Point3f& angles);
    
    // Getters
    cv::Point3f getPosition() const { return position_; }
    cv::Point3f getTarget() const { return target_; }
    cv::Point3f getUp() const { return up_; }
    CameraMode getMode() const { return mode_; }
    
    // Camera settings
    static constexpr float THIRD_PERSON_DISTANCE = 200.0f;  // Distance behind drone
    static constexpr float THIRD_PERSON_HEIGHT = 100.0f;    // Height above drone
    static constexpr float MIN_ZOOM = 50.0f;               // Closest zoom
    static constexpr float MAX_ZOOM = 500.0f;              // Farthest zoom

private:
    cv::Point3f position_;      // Camera position
    cv::Point3f target_;        // Where camera is looking
    cv::Point3f up_;            // Up vector
    CameraMode mode_;           // Current camera mode
    
    // Third-person camera state
    float orbit_angle_x_;       // Horizontal orbit angle
    float orbit_angle_y_;       // Vertical orbit angle
    float zoom_distance_;       // Current zoom distance
    
    // Helper methods
    void updateThirdPersonPosition(const cv::Point3f& drone_pos, const cv::Point3f& drone_orientation);
    cv::Point3f calculateOrbitPosition(float angle_x, float angle_y, float distance);
};
