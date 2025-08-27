#include "agent/Agent.h"
#include "sim/Drone.h"
#include <algorithm>

namespace agent {

Agent::Agent(const AgentConfig& config) : config_(config) {
}

std::vector<Action> Agent::getValidActions(const sim::Drone& drone) const {
    std::vector<Action> valid_actions;
    
    // Basic 2D actions (backward compatibility)
    std::vector<Action> basic_actions = {Action::THROTTLE_FORWARD, Action::YAW_LEFT, Action::YAW_RIGHT, Action::IDLE};
    
    // 3D actions (if 3D mode is enabled)
    std::vector<Action> three_d_actions = {Action::PITCH_UP, Action::PITCH_DOWN, Action::ROLL_LEFT, Action::ROLL_RIGHT, 
                                          Action::THRUST_UP, Action::THRUST_DOWN};
    
    // Combined 3D actions
    std::vector<Action> combined_actions = {Action::FORWARD_AND_UP, Action::FORWARD_AND_DOWN, 
                                           Action::TURN_AND_CLIMB, Action::TURN_AND_DIVE};
    
    // Add all actions if 3D mode is enabled
    if (config_.use_3d) {
        valid_actions.insert(valid_actions.end(), three_d_actions.begin(), three_d_actions.end());
        valid_actions.insert(valid_actions.end(), combined_actions.begin(), combined_actions.end());
    }
    
    // Always include basic actions
    valid_actions.insert(valid_actions.end(), basic_actions.begin(), basic_actions.end());
    
    // Check each action for validity
    std::vector<Action> filtered_actions;
    for (const auto& action : valid_actions) {
        if (isValidAction(action, drone)) {
            filtered_actions.push_back(action);
        }
    }
    
    // Always include IDLE as a fallback
    if (std::find(filtered_actions.begin(), filtered_actions.end(), Action::IDLE) == filtered_actions.end()) {
        filtered_actions.push_back(Action::IDLE);
    }
    
    return filtered_actions;
}

bool Agent::isValidAction(Action action, const sim::Drone& drone) const {
    // Convert action to control inputs
    float throttle = 0.0f;
    float yaw_rate = 0.0f;
    float pitch_rate = 0.0f;
    float roll_rate = 0.0f;
    float vertical_thrust = 0.0f;
    
    switch (action) {
        // Basic 2D actions
        case Action::THROTTLE_FORWARD:
            throttle = 1.0f;
            yaw_rate = 0.0f;
            break;
        case Action::YAW_LEFT:
            throttle = 0.0f;
            yaw_rate = -1.0f;
            break;
        case Action::YAW_RIGHT:
            throttle = 0.0f;
            yaw_rate = 1.0f;
            break;
        case Action::IDLE:
            throttle = 0.0f;
            yaw_rate = 0.0f;
            break;
            
        // 3D actions
        case Action::PITCH_UP:
            throttle = 0.0f;
            yaw_rate = 0.0f;
            pitch_rate = 1.0f;
            break;
        case Action::PITCH_DOWN:
            throttle = 0.0f;
            yaw_rate = 0.0f;
            pitch_rate = -1.0f;
            break;
        case Action::ROLL_LEFT:
            throttle = 0.0f;
            yaw_rate = 0.0f;
            roll_rate = -1.0f;
            break;
        case Action::ROLL_RIGHT:
            throttle = 0.0f;
            yaw_rate = 0.0f;
            roll_rate = 1.0f;
            break;
        case Action::THRUST_UP:
            throttle = 0.0f;
            yaw_rate = 0.0f;
            vertical_thrust = 1.0f;
            break;
        case Action::THRUST_DOWN:
            throttle = 0.0f;
            yaw_rate = 0.0f;
            vertical_thrust = -1.0f;
            break;
            
        // Combined 3D actions
        case Action::FORWARD_AND_UP:
            throttle = 1.0f;
            yaw_rate = 0.0f;
            vertical_thrust = 1.0f;
            break;
        case Action::FORWARD_AND_DOWN:
            throttle = 1.0f;
            yaw_rate = 0.0f;
            vertical_thrust = -1.0f;
            break;
        case Action::TURN_AND_CLIMB:
            throttle = 0.0f;
            yaw_rate = 1.0f;
            vertical_thrust = 1.0f;
            break;
        case Action::TURN_AND_DIVE:
            throttle = 0.0f;
            yaw_rate = 1.0f;
            vertical_thrust = -1.0f;
            break;
    }
    
    // Check if action is within drone constraints
    if (!drone.isWithinConstraints(throttle, yaw_rate, pitch_rate, roll_rate, vertical_thrust)) {
        return false;
    }
    
    // Check if action would lead to collision
    if (drone.wouldCollide(throttle, yaw_rate, pitch_rate, roll_rate, vertical_thrust, 0.1f)) { // 0.1s time step
        return false;
    }
    
    return true;
}

} // namespace agent
