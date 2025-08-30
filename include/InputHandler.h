#pragma once

#include "Drone.h"
#include "Camera.h"

class InputHandler {
public:
    InputHandler();
    
    // Input processing
    void processKey(int key, bool pressed);
    void processMouse(double xpos, double ypos);
    
    // Input updates
    void update(float delta_time);
    
    // Camera setup
    void setCamera(Camera* camera) { camera_ = camera; }
    
    // Getters
    DroneInput getCurrentInput() const { return current_input_; }
    
    // State queries
    bool isExitRequested() const { return exit_requested_; }
    bool isCameraModeChanged() const { return camera_mode_changed_; }
    bool isPauseRequested() const { return pause_requested_; }
    
    // Camera controls
    void orbit(float delta_x, float delta_y);
    void zoom(float delta);
    void resetView();
    
    // Camera state
    float getOrbitAngleX() const { return orbit_angle_x_; }
    float getOrbitAngleY() const { return orbit_angle_y_; }
    float getZoomDistance() const { return zoom_distance_; }

private:
    DroneInput current_input_;
    Camera* camera_;  // Reference to camera for immediate mode switching
    
    // Input state
    bool exit_requested_;
    bool camera_mode_changed_;
    bool pause_requested_;
    
    // Debounce for camera mode switching
    bool space_key_pressed_;
    float last_space_press_time_;
    
    // Camera state
    float orbit_angle_x_;
    float orbit_angle_y_;
    float zoom_distance_;
    
    // Constants
    static constexpr float ORBIT_SENSITIVITY = 0.002f;  // Further reduced for much slower camera movement
    static constexpr float ZOOM_SENSITIVITY = 2.0f;     // Further reduced for much slower zooming
    static constexpr float MIN_ZOOM = 20.0f;
    static constexpr float MAX_ZOOM = 200.0f;
    
    // Additional sensitivity controls
    static constexpr float MOUSE_DEADZONE = 1.0f;       // Minimum mouse movement to register
    static constexpr float DRONE_ROLL_SENSITIVITY = 1.0f;   // Can be adjusted for drone roll speed
    static constexpr float DRONE_YAW_SENSITIVITY = 0.5f;    // Reduced yaw sensitivity for slower turning
    static constexpr float DRONE_PITCH_SENSITIVITY = 1.0f;  // Can be adjusted for drone pitch speed
    
    // Debounce timing
    static constexpr float SPACE_DEBOUNCE_TIME = 0.2f;   // Minimum time between space key presses (seconds)
};
