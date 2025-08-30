#pragma once

#include "Drone.h"

class InputHandler {
public:
    InputHandler();
    
    // Input processing
    void processKey(int key, bool pressed);
    void processMouse(double xpos, double ypos);
    
    // Input updates
    void update(float delta_time);
    
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
    
    // Input state
    bool exit_requested_;
    bool camera_mode_changed_;
    bool pause_requested_;
    
    // Camera state
    float orbit_angle_x_;
    float orbit_angle_y_;
    float zoom_distance_;
    
    // Constants
    static constexpr float ORBIT_SENSITIVITY = 0.01f;
    static constexpr float ZOOM_SENSITIVITY = 10.0f;
    static constexpr float MIN_ZOOM = 20.0f;
    static constexpr float MAX_ZOOM = 200.0f;
};
