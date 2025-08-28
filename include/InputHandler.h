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
    void processMouse(int event, int x, int y, int flags);
    void update();
    
    // Getters
    const DroneInput& getDroneInput() const { return drone_input_; }
    bool shouldExit() const { return exit_requested_; }
    bool shouldToggleCamera() const { return camera_toggle_requested_; }
    bool shouldPause() const { return pause_toggle_requested_; }
    
    // Reset flags
    void clearCameraToggle() { camera_toggle_requested_ = false; }
    void clearPauseToggle() { pause_toggle_requested_ = false; }
    
    // Input sensitivity
    static constexpr float INPUT_SENSITIVITY = 0.1f;     // How responsive controls are
    static constexpr float MAX_INPUT_RATE = 2.0f;        // Max input change per second

private:
    DroneInput drone_input_;
    bool exit_requested_;
    bool camera_toggle_requested_;
    bool pause_toggle_requested_;
    
    // Mouse state for third-person camera
    cv::Point2i last_mouse_pos_;
    bool mouse_dragging_;
    
    // Input smoothing
    DroneInput target_input_;
    DroneInput current_input_;
    
    // Helper methods
    void updateInputSmoothing(float delta_time);
    void handleKeyPress(int key);
    void handleKeyRelease(int key);
};
