#pragma once

#include "Drone.h"
#include <cmath>

enum class CameraMode {
    FIRST_PERSON,
    THIRD_PERSON
};

struct CameraState {
    float x, y, z;           // Position
    float target_x, target_y, target_z;  // Look target
    float up_x, up_y, up_z;  // Up vector
};

class Camera {
public:
    Camera();
    
    // Camera mode switching
    void setFirstPersonMode(const struct DroneState& drone_pos, const struct DroneState& drone_orient);
    void setThirdPersonMode(const struct DroneState& drone_pos, const struct DroneState& drone_orient);
    
    // Camera updates
    void update(const struct DroneState& drone_pos, const struct DroneState& drone_orient);
    
    // Camera controls
    void orbit(float delta_x, float delta_y);
    void zoom(float delta);
    void resetView();
    
    // Getters
    CameraMode getMode() const { return mode_; }
    CameraState getPosition() const { return position_; }
    CameraState getTarget() const { return target_; }
    CameraState getUp() const { return up_; }
    
    // Helper methods
    CameraState rotateVector(const CameraState& vec, const CameraState& angles);

private:
    CameraMode mode_;
    CameraState position_;
    CameraState target_;
    CameraState up_;
    
    // Third-person camera parameters
    float orbit_angle_x_;
    float orbit_angle_y_;
    float zoom_distance_;
    
    // Constants
    static constexpr float THIRD_PERSON_HEIGHT = 50.0f;
    static constexpr float ORBIT_SENSITIVITY = 0.002f;  // Further reduced for much slower camera movement
    static constexpr float ZOOM_SENSITIVITY = 2.0f;     // Further reduced for much slower zooming
    static constexpr float MIN_ZOOM = 20.0f;
    static constexpr float MAX_ZOOM = 200.0f;
    static constexpr float CAMERA_ROLL_SENSITIVITY = 0.3f;  // Reduce camera roll effect from drone roll
    
    // Helper methods
    void updateFirstPersonPosition(const struct DroneState& drone_pos, const struct DroneState& drone_orient);
    void updateThirdPersonPosition(const struct DroneState& drone_pos, const struct DroneState& drone_orient);
    CameraState calculateOrbitPosition(float angle_x, float angle_y, float distance);
};
