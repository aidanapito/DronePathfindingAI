#include "InputHandler.h"
#include <algorithm>

InputHandler::InputHandler() {
    // Initialize all inputs to zero
    drone_input_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    target_input_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    current_input_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    
    // Initialize flags
    exit_requested_ = false;
    camera_toggle_requested_ = false;
    pause_toggle_requested_ = false;
    
    // Initialize mouse state
    last_mouse_pos_ = cv::Point2i(0, 0);
    mouse_dragging_ = false;
}

void InputHandler::processKey(int key, bool pressed) {
    if (pressed) {
        handleKeyPress(key);
    } else {
        handleKeyRelease(key);
    }
}

void InputHandler::processMouse(int event, int x, int y, int flags) {
    switch (event) {
        case cv::EVENT_MOUSEMOVE:
            if (mouse_dragging_) {
                // Calculate mouse movement delta
                int delta_x = x - last_mouse_pos_.x;
                int delta_y = y - last_mouse_pos_.y;
                
                // Store current position for next frame
                last_mouse_pos_ = cv::Point2i(x, y);
            }
            break;
            
        case cv::EVENT_LBUTTONDOWN:
            mouse_dragging_ = true;
            last_mouse_pos_ = cv::Point2i(x, y);
            break;
            
        case cv::EVENT_LBUTTONUP:
            mouse_dragging_ = false;
            break;
            
        case cv::EVENT_MOUSEWHEEL:
            // Handle mouse wheel for zooming
            break;
    }
}

void InputHandler::update() {
    // Update input smoothing
    updateInputSmoothing(0.016f); // Assume 60 FPS
    
    // Apply smoothed input to drone input
    drone_input_ = current_input_;
}

void InputHandler::updateInputSmoothing(float delta_time) {
    // Smoothly interpolate current input toward target input
    float smoothing_factor = std::min(MAX_INPUT_RATE * delta_time, 1.0f);
    
    current_input_.throttle = current_input_.throttle + 
                             (target_input_.throttle - current_input_.throttle) * smoothing_factor;
    current_input_.yaw_rate = current_input_.yaw_rate + 
                             (target_input_.yaw_rate - current_input_.yaw_rate) * smoothing_factor;
    current_input_.pitch_rate = current_input_.pitch_rate + 
                               (target_input_.pitch_rate - current_input_.pitch_rate) * smoothing_factor;
    current_input_.roll_rate = current_input_.roll_rate + 
                              (target_input_.roll_rate - current_input_.roll_rate) * smoothing_factor;
    current_input_.vertical_thrust = current_input_.vertical_thrust + 
                                    (target_input_.vertical_thrust - current_input_.vertical_thrust) * smoothing_factor;
}

void InputHandler::handleKeyPress(int key) {
    switch (key) {
        case 'w':
        case 'W':
            target_input_.throttle = 1.0f; // Forward
            break;
        case 's':
        case 'S':
            target_input_.throttle = -1.0f; // Backward
            break;
        case 'a':
        case 'A':
            target_input_.roll_rate = -1.0f; // Roll left
            break;
        case 'd':
        case 'D':
            target_input_.roll_rate = 1.0f; // Roll right
            break;
        case 'q':
        case 'Q':
            target_input_.yaw_rate = -1.0f; // Turn left
            break;
        case 'e':
        case 'E':
            target_input_.yaw_rate = 1.0f; // Turn right
            break;
        case 'r':
        case 'R':
            target_input_.pitch_rate = 1.0f; // Pitch up
            break;
        case 'f':
        case 'F':
            target_input_.pitch_rate = -1.0f; // Pitch down
            break;
        case ' ':
            pause_toggle_requested_ = true;
            break;
        case 'c':
        case 'C':
            camera_toggle_requested_ = true;
            break;
        case 27: // ESC key
            exit_requested_ = true;
            break;
    }
}

void InputHandler::handleKeyRelease(int key) {
    switch (key) {
        case 'w':
        case 'W':
        case 's':
        case 'S':
            target_input_.throttle = 0.0f;
            break;
        case 'a':
        case 'A':
        case 'd':
        case 'D':
            target_input_.roll_rate = 0.0f;
            break;
        case 'q':
        case 'Q':
        case 'e':
        case 'E':
            target_input_.yaw_rate = 0.0f;
            break;
        case 'r':
        case 'R':
        case 'f':
        case 'F':
            target_input_.pitch_rate = 0.0f;
            break;
    }
}
