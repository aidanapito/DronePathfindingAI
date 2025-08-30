#include "InputHandler.h"
#include <iostream>
#include <algorithm>

InputHandler::InputHandler() {
    current_input_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    exit_requested_ = false;
    camera_mode_changed_ = false;
    pause_requested_ = false;
    orbit_angle_x_ = 0.0f;
    orbit_angle_y_ = 0.0f;
    zoom_distance_ = 100.0f;
    camera_ = nullptr;
}

void InputHandler::processKey(int key, bool pressed) {
    if (!pressed) {
        // Reset input when key is released
        switch (key) {
            case 'w': case 'W':
            case 's': case 'S':
                current_input_.forward_thrust = 0.0f;
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
        return;
    }
    
    // Handle key press
    switch (key) {
        case 'w': case 'W':
            current_input_.forward_thrust = 1.0f; // Full forward
            break;
        case 's': case 'S':
            current_input_.forward_thrust = -1.0f; // Full backward
            break;
        case 'a': case 'A':
            current_input_.roll_rate = 1.0f; // Roll left
            break;
        case 'd': case 'D':
            current_input_.roll_rate = -1.0f; // Roll right
            break;
        case 'q': case 'Q':
            current_input_.yaw_rate = 1.0f * DRONE_YAW_SENSITIVITY; // Yaw left (with sensitivity)
            break;
        case 'e': case 'E':
            current_input_.yaw_rate = -1.0f * DRONE_YAW_SENSITIVITY; // Yaw right (with sensitivity)
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
            if (camera_) {
                // Immediately switch camera mode to prevent switching loop
                if (camera_->getMode() == CameraMode::FIRST_PERSON) {
                    camera_->setThirdPersonMode({0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0});
                    std::cout << "Switched to THIRD_PERSON mode" << std::endl;
                } else {
                    camera_->setFirstPersonMode({0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0});
                    std::cout << "Switched to FIRST_PERSON mode" << std::endl;
                }
            } else {
                std::cout << "Warning: Camera not set in InputHandler" << std::endl;
            }
            break;
        case 27: // ESC key
            exit_requested_ = true;
            break;
        case 'p': case 'P':
            pause_requested_ = true;
            break;
    }
}

void InputHandler::processMouse(double xpos, double ypos) {
    // Mouse movement for camera control
    static double last_x = xpos;
    static double last_y = ypos;
    
    double delta_x = xpos - last_x;
    double delta_y = ypos - last_y;
    
    last_x = xpos;
    last_y = ypos;
    
    // Only apply mouse movement if there's significant movement (reduce jitter)
    if (std::abs(delta_x) > MOUSE_DEADZONE || std::abs(delta_y) > MOUSE_DEADZONE) {
        // Apply mouse movement to camera
        orbit(delta_x, delta_y);
    }
}

void InputHandler::update(float delta_time) {
    // Reset camera mode change flag
    camera_mode_changed_ = false;
    pause_requested_ = false;
}

void InputHandler::orbit(float delta_x, float delta_y) {
    orbit_angle_x_ += delta_x * ORBIT_SENSITIVITY;
    orbit_angle_y_ += delta_y * ORBIT_SENSITIVITY;
    
    // Clamp vertical angle
    orbit_angle_y_ = std::max(-static_cast<float>(M_PI)/3.0f, std::min(static_cast<float>(M_PI)/3.0f, orbit_angle_y_));
    
    // Keep horizontal angle in range
    orbit_angle_x_ = std::fmod(orbit_angle_x_, 2.0f*static_cast<float>(M_PI));
}

void InputHandler::zoom(float delta) {
    zoom_distance_ += delta * ZOOM_SENSITIVITY;
    zoom_distance_ = std::max(MIN_ZOOM, std::min(MAX_ZOOM, zoom_distance_));
}

void InputHandler::resetView() {
    orbit_angle_x_ = 0.0f;
    orbit_angle_y_ = 0.0f;
    zoom_distance_ = 100.0f;
}
