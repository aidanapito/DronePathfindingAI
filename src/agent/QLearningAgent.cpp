#include "agent/QLearningAgent.h"
#include <random>

namespace agent {

QLearningAgent::QLearningAgent(const AgentConfig& config) 
    : Agent(config), alpha_(0.1f), gamma_(0.99f), epsilon_(0.1f),
      grid_resolution_(20), num_heading_buckets_(8) {
}

Action QLearningAgent::selectAction(const Observation& obs) {
    QState state = discretizeState(obs);
    
    if (epsilon_ > 0.0f && (static_cast<float>(rand()) / RAND_MAX) < epsilon_) {
        return epsilonGreedyAction(state);
    } else {
        return greedyAction(state);
    }
}

void QLearningAgent::updatePolicy(const Observation& obs, Action action, 
                                  float reward, const Observation& next_obs, bool done) {
    QState state = discretizeState(obs);
    QState next_state = discretizeState(next_obs);
    
    updateQValue(state, action, reward, next_state, done);
}

void QLearningAgent::reset() {
    // Reset episode-specific state if needed
}

void QLearningAgent::updateQValue(const QState& state, Action action, 
                                  float reward, const QState& next_state, bool done) {
    initializeQValue(state);
    
    float current_q = getQValue(state, action);
    float max_next_q = done ? 0.0f : getMaxQValue(next_state);
    
    float new_q = current_q + alpha_ * (reward + gamma_ * max_next_q - current_q);
    setQValue(state, action, new_q);
}

float QLearningAgent::getQValue(const QState& state, Action action) const {
    auto it = q_table_.find(state);
    if (it == q_table_.end()) {
        return 0.0f;
    }
    
    int action_idx = static_cast<int>(action);
    if (action_idx >= 0 && action_idx < it->second.size()) {
        return it->second[action_idx];
    }
    
    return 0.0f;
}

void QLearningAgent::setQValue(const QState& state, Action action, float value) {
    initializeQValue(state);
    
    int action_idx = static_cast<int>(action);
    if (action_idx >= 0 && action_idx < q_table_[state].size()) {
        q_table_[state][action_idx] = value;
    }
}

void QLearningAgent::saveModel(const std::string& path) {
    // TODO: Implement model saving
}

void QLearningAgent::loadModel(const std::string& path) {
    // TODO: Implement model loading
}

float QLearningAgent::getAverageQValue() const {
    if (q_table_.empty()) return 0.0f;
    
    float total = 0.0f;
    int count = 0;
    
    for (const auto& entry : q_table_) {
        for (float q_value : entry.second) {
            total += q_value;
            count++;
        }
    }
    
    return count > 0 ? total / count : 0.0f;
}

QState QLearningAgent::discretizeState(const Observation& obs) const {
    QState state;
    
    // Discretize grid position (assuming 21x21 grid)
    auto grid_pos = worldToGrid(cv::Point2f(0, 0)); // TODO: Get actual position
    state.grid_x = grid_pos.first;
    state.grid_y = grid_pos.second;
    
    // Discretize heading
    state.heading_bucket = headingToBucket(obs.heading);
    
    return state;
}

Action QLearningAgent::epsilonGreedyAction(const QState& state) {
    // Random action for exploration
    std::vector<Action> actions = {Action::THROTTLE_FORWARD, Action::YAW_LEFT, 
                                   Action::YAW_RIGHT, Action::IDLE};
    return actions[rand() % actions.size()];
}

Action QLearningAgent::greedyAction(const QState& state) {
    // Best action based on Q-values
    initializeQValue(state);
    
    const auto& q_values = q_table_[state];
    int best_action = 0;
    float best_value = q_values[0];
    
    for (int i = 1; i < q_values.size(); ++i) {
        if (q_values[i] > best_value) {
            best_value = q_values[i];
            best_action = i;
        }
    }
    
    return static_cast<Action>(best_action);
}

void QLearningAgent::initializeQValue(const QState& state) {
    if (q_table_.find(state) == q_table_.end()) {
        q_table_[state] = std::vector<float>(4, 0.0f); // 4 actions
    }
}

std::pair<int, int> QLearningAgent::worldToGrid(const cv::Point2f& world_pos) const {
    // TODO: Implement proper world to grid conversion
    return {static_cast<int>(world_pos.x / grid_resolution_), 
            static_cast<int>(world_pos.y / grid_resolution_)};
}

int QLearningAgent::headingToBucket(float heading) const {
    // Convert heading to bucket (0-7 for 45° increments)
    float normalized = fmod(heading + M_PI, 2.0f * M_PI);
    int bucket = static_cast<int>((normalized / (2.0f * M_PI)) * num_heading_buckets_);
    return std::min(bucket, num_heading_buckets_ - 1);
}

float QLearningAgent::getMaxQValue(const QState& state) const {
    auto it = q_table_.find(state);
    if (it == q_table_.end()) return 0.0f;
    
    float max_q = it->second[0];
    for (float q_value : it->second) {
        max_q = std::max(max_q, q_value);
    }
    
    return max_q;
}

} // namespace agent
