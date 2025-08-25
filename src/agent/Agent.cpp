#include "agent/Agent.h"
#include "sim/Drone.h"
#include <algorithm>

namespace agent {

Agent::Agent(const AgentConfig& config) : config_(config) {
}

std::vector<Action> Agent::getValidActions(const sim::Drone& drone) const {
    std::vector<Action> valid_actions;
    std::vector<Action> all_actions = {Action::THROTTLE_FORWARD, Action::YAW_LEFT, Action::YAW_RIGHT, Action::IDLE};
    
    // Check each action for validity
    for (const auto& action : all_actions) {
        if (isValidAction(action, drone)) {
            valid_actions.push_back(action);
        }
    }
    
    // Always include IDLE as a fallback
    if (std::find(valid_actions.begin(), valid_actions.end(), Action::IDLE) == valid_actions.end()) {
        valid_actions.push_back(Action::IDLE);
    }
    
    return valid_actions;
}

bool Agent::isValidAction(Action action, const sim::Drone& drone) const {
    // Convert action to throttle and yaw_rate
    float throttle = 0.0f;
    float yaw_rate = 0.0f;
    
    switch (action) {
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
    }
    
    // Check if action is within drone constraints
    if (!drone.isWithinConstraints(throttle, yaw_rate)) {
        return false;
    }
    
    // Check if action would lead to collision
    if (drone.wouldCollide(throttle, yaw_rate, 0.1f)) { // 0.1s time step
        return false;
    }
    
    return true;
}

} // namespace agent
