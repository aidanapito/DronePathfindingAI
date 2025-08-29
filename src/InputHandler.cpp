#include "InputHandler.h"
#include <iostream>

InputHandler::InputHandler() {
    // Initialize all inputs to 0
    current_input_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    target_input_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    
    // Initialize flags
    exit_requested_ = false;
    camera_mode_changed_ = false;
    pause_requested_ = false;
    
    // Initialize camera control
    orbit_angle_x_ = 0.0f;
    orbit_angle_y_ = 0.0f;
    zoom_distance_ = 200.0f;
}

void InputHandler::processKey(int key, bool pressed) {
    if (pressed) {
        // Key pressed - set input values
        switch (key) {
            case 'w': case 'W':
                current_input_.throttle = 1.0f; // Full forward
                break;
            case 's': case 'S':
                current_input_.throttle = -1.0f; // Full backward
                break;
            case 'a': case 'A':
                current_input_.roll_rate = -1.0f; // Roll left
                break;
            case 'd': case 'D':
                current_input_.roll_rate = 1.0f; // Roll right
                break;
            case 'q': case 'Q':
                current_input_.yaw_rate = -1.0f; // Turn left
                break;
            case 'e': case 'E':
                current_input_.yaw_rate = 1.0f; // Turn right
                break;
            case 'r': case 'R':
                current_input_.pitch_rate = 1.0f; // Pitch up
                break;
            case 'f': case 'F':
                current_input_.pitch_rate = -1.0f; // Pitch down
                break;
            case 'z': case 'Z':
                current_input_.vertical_thrust = 1.0f; // Fly up
                break;
            case 'x': case 'X':
                current_input_.vertical_thrust = -1.0f; // Fly down
                break;
            case ' ': // Spacebar
                pause_requested_ = true;
                break;
            case 'c': case 'C':
                camera_mode_changed_ = true;
                break;
            case 27: // ESC
                exit_requested_ = true;
                break;
        }
    } else {
        // Key released - reset input values
        switch (key) {
            case 'w': case 'W':
            case 's': case 'S':
                current_input_.throttle = 0.0f;
                break;
            case 'a': case 'A':
            case 'd': case 'D':
                current_input_.roll_rate = 0.0f;
                break;
            case 'q': case 'Q':
            case 'e': case 'E':
                current_input_.yaw_rate = 0.0f;
                break;
            case 'r': case 'R':
            case 'f': case 'F':
                current_input_.pitch_rate = 0.0f;
                break;
            case 'z': case 'Z':
            case 'x': case 'X':
                current_input_.vertical_thrust = 0.0f;
                break;
        }
    }
}

void InputHandler::processMouse(int x, int y, int flags) {
    // Mouse handling for camera control
    if (flags & cv::EVENT_FLAG_LBUTTON) {
        // Left mouse button - orbit camera
        orbit(x, y);
    }
}

void InputHandler::update(float delta_time) {
    // No smoothing needed - use direct values for immediate response
    
    // Reset flags
    camera_mode_changed_ = false;
    pause_requested_ = false;
}

void InputHandler::orbit(int delta_x, int delta_y) {
    // Simple orbit calculation
    orbit_angle_x_ += delta_x * 0.01f;
    orbit_angle_y_ += delta_y * 0.01f;
    
    // Clamp vertical angle
    if (orbit_angle_y_ > M_PI / 3.0f) orbit_angle_y_ = M_PI / 3.0f;
    if (orbit_angle_y_ < -M_PI / 3.0f) orbit_angle_y_ = -M_PI / 3.0f;
    
    // Wrap horizontal angle
    orbit_angle_x_ = std::fmod(orbit_angle_x_, 2.0f * M_PI);
}

void InputHandler::zoom(float delta) {
    zoom_distance_ += delta * 10.0f;
    if (zoom_distance_ < 50.0f) zoom_distance_ = 50.0f;
    if (zoom_distance_ > 500.0f) zoom_distance_ = 500.0f;
}

void InputHandler::resetView() {
    orbit_angle_x_ = 0.0f;
    orbit_angle_y_ = 0.0f;
    zoom_distance_ = 200.0f;
}

// Getters
const DroneInput& InputHandler::getCurrentInput() const { return current_input_; }
bool InputHandler::isExitRequested() const { return exit_requested_; }
bool InputHandler::isCameraModeChanged() const { return camera_mode_changed_; }
bool InputHandler::isPauseRequested() const { return pause_requested_; }
float InputHandler::getOrbitAngleX() const { return orbit_angle_x_; }
float InputHandler::getOrbitAngleY() const { return orbit_angle_y_; }
float InputHandler::getZoomDistance() const { return zoom_distance_; }
