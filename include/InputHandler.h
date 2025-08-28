#pragma once

#include <opencv2/opencv.hpp>
#include "Camera.h"

struct DroneInput {
    float throttle;      // Forward/backward movement (-1.0 to 1.0)
    float yaw_rate;      // Left/right rotation (-1.0 to 1.0)
    float pitch_rate;    // Up/down pitch (-1.0 to 1.0)
    float roll_rate;     // Left/right roll (-1.0 to 1.0)
    float vertical_thrust; // Up/down movement (-1.0 to 1.0)
};

class InputHandler {
public:
    InputHandler();
    
    // Input processing
    void processKey(int key, bool pressed);
    void processMouse(int x, int y, int flags);
    void update(float delta_time);
    
    // Getters
    const DroneInput& getCurrentInput() const;
    bool isExitRequested() const;
    bool isCameraModeChanged() const;
    bool isPauseRequested() const;
    
    // Camera control
    void orbit(int delta_x, int delta_y);
    void zoom(float delta);
    void resetView();
    
    // Camera getters
    float getOrbitAngleX() const;
    float getOrbitAngleY() const;
    float getZoomDistance() const;

private:
    DroneInput current_input_;
    DroneInput target_input_;
    bool exit_requested_;
    bool camera_mode_changed_;
    bool pause_requested_;
    
    // Camera control
    float orbit_angle_x_;
    float orbit_angle_y_;
    float zoom_distance_;
};
