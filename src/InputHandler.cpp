#include "InputHandler.h"
#include "AI/PathfindingAI.h"
#include <iostream>
#include <algorithm>

InputHandler::InputHandler() {
    current_input_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    exit_requested_ = false;
    camera_mode_changed_ = false;
    pause_requested_ = false;
    should_switch_camera_mode_ = false;
    orbit_angle_x_ = 0.0f;
    orbit_angle_y_ = 0.0f;
    zoom_distance_ = 100.0f;
    camera_ = nullptr;
    ai_control_enabled_ = false;
    ai_controller_ = nullptr;
}

void InputHandler::processKey(int key, bool pressed) {
    // Update key state
    key_states_[key] = pressed;
    
    // Handle immediate actions that don't depend on continuous input
    if (pressed) {
        switch (key) {
            case ' ': // Spacebar - immediate camera mode switch
                should_switch_camera_mode_ = true;
                std::cout << "Camera mode switch requested" << std::endl;
                break;
            case 27: // ESC key
                exit_requested_ = true;
                break;
            case 'p': case 'P':
                pause_requested_ = true;
                break;
            case '1': // AI Mode 1: Manual
                setAIMode(0);
                break;
            case '2': // AI Mode 2: Follow Path
                setAIMode(1);
                break;
            case '3': // AI Mode 3: Explore
                setAIMode(2);
                break;
            case '4': // AI Mode 4: Return Home
                setAIMode(3);
                break;
            case '5': // AI Mode 5: Avoid Obstacles
                setAIMode(4);
                break;
            case 't': case 'T': // Toggle AI control
                setAIControl(!ai_control_enabled_);
                std::cout << "AI Control " << (ai_control_enabled_ ? "ENABLED" : "DISABLED") << std::endl;
                break;
        }
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
        // Apply mouse movement to camera if available
        if (camera_) {
            camera_->handleMouseInput(delta_x, delta_y);
        } else {
            // Fallback to local orbit if camera not available
            orbit(delta_x, delta_y);
        }
    }
}

void InputHandler::update(float delta_time) {
    // Reset camera mode change flag
    camera_mode_changed_ = false;
    pause_requested_ = false;
    
    // Calculate continuous input based on current key states
    current_input_.forward_thrust = 0.0f;
    current_input_.roll_rate = 0.0f;
    current_input_.yaw_rate = 0.0f;
    current_input_.pitch_rate = 0.0f;
    current_input_.vertical_thrust = 0.0f;
    
    // Check W/S keys for forward/backward thrust
    if (key_states_['w'] || key_states_['W']) {
        current_input_.forward_thrust = 1.0f;
    }
    if (key_states_['s'] || key_states_['S']) {
        current_input_.forward_thrust = -1.0f;
    }
    
    // Check A/D keys for roll
    if (key_states_['a'] || key_states_['A']) {
        current_input_.roll_rate = 1.0f;
    }
    if (key_states_['d'] || key_states_['D']) {
        current_input_.roll_rate = -1.0f;
    }
    
    // Check Q/E keys for yaw
    if (key_states_['q'] || key_states_['Q']) {
        current_input_.yaw_rate = 1.0f * DRONE_YAW_SENSITIVITY;
    }
    if (key_states_['e'] || key_states_['E']) {
        current_input_.yaw_rate = -1.0f * DRONE_YAW_SENSITIVITY;
    }
    
    // Check R/F keys for pitch
    if (key_states_['r'] || key_states_['R']) {
        current_input_.pitch_rate = 1.0f;
    }
    if (key_states_['f'] || key_states_['F']) {
        current_input_.pitch_rate = -1.0f;
    }
    
    // Check Z/X keys for vertical thrust
    if (key_states_['z'] || key_states_['Z']) {
        current_input_.vertical_thrust = 1.0f;
    }
    if (key_states_['x'] || key_states_['X']) {
        current_input_.vertical_thrust = -1.0f;
    }
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

void InputHandler::setAITarget(float x, float y, float z) {
    if (ai_controller_) {
        ai_controller_->setTarget(x, y, z);
        std::cout << "AI Target set to: (" << x << ", " << y << ", " << z << ")" << std::endl;
    }
}

void InputHandler::setAIMode(int mode) {
    if (ai_controller_) {
        switch (mode) {
            case 0: ai_controller_->setMode(AIMode::MANUAL); break;
            case 1: ai_controller_->setMode(AIMode::FOLLOW_PATH); break;
            case 2: ai_controller_->setMode(AIMode::EXPLORE); break;
            case 3: ai_controller_->setMode(AIMode::RETURN_HOME); break;
            case 4: ai_controller_->setMode(AIMode::AVOID_OBSTACLES); break;
        }
    }
}
