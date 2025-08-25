#include "agent/Agent.h"

namespace agent {

Agent::Agent(const AgentConfig& config) : config_(config) {
}

std::vector<Action> Agent::getValidActions(const sim::Drone& drone) const {
    // Return all actions for now - can be customized per agent
    return {Action::THROTTLE_FORWARD, Action::YAW_LEFT, Action::YAW_RIGHT, Action::IDLE};
}

bool Agent::isValidAction(Action action, const sim::Drone& drone) const {
    // Basic validation - can be enhanced
    return true;
}

} // namespace agent
