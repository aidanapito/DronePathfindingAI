#include "bridge/Environment.h"

namespace bridge {

Environment::Environment(const EnvironmentConfig& config) 
    : config_(config), current_step_(0), cumulative_reward_(0.0f) {
}

void Environment::reset() {
    current_step_ = 0;
    cumulative_reward_ = 0.0f;
    path_trace_.clear();
    
    if (world_ && drone_) {
        episode_start_ = drone_->getState().position;
        episode_goal_ = world_->getGoalPosition();
    }
}

EpisodeResult Environment::runEpisode(std::shared_ptr<agent::Agent> agent) {
    reset();
    
    while (!isDone()) {
        step(agent);
    }
    
    EpisodeResult result;
    result.steps = current_step_;
    result.total_reward = cumulative_reward_;
    result.success = drone_ && world_ && 
                    drone_->hasReachedGoal(world_->getGoalPosition());
    result.final_distance = drone_ && world_ ? 
                           drone_->getDistanceToGoal(world_->getGoalPosition()) : 0.0f;
    result.path_trace = path_trace_;
    
    episode_history_.push_back(result);
    return result;
}

void Environment::step(std::shared_ptr<agent::Agent> agent) {
    if (!agent || !world_ || !drone_) return;
    
    // Get current observation
    agent::Observation obs = getCurrentObservation();
    
    // Agent selects action
    agent::Action action = agent->selectAction(obs);
    
    // TODO: Execute action and get next observation
    agent::Observation next_obs = obs; // Placeholder
    
    // Calculate reward
    float reward = getCurrentReward();
    cumulative_reward_ += reward;
    
    // Update agent policy
    agent->updatePolicy(obs, action, reward, next_obs, isDone());
    
    // Update path trace
    updatePathTrace();
    
    current_step_++;
}

bool Environment::isDone() const {
    if (!world_ || !drone_) return true;
    
    // Check termination conditions
    if (current_step_ >= config_.max_steps_per_episode) return true;
    if (drone_->hasReachedGoal(world_->getGoalPosition())) return true;
    if (world_->checkCollision(drone_->getState().position, 10.0f)) return true;
    
    return false;
}

float Environment::getCurrentReward() const {
    return calculateReward();
}

agent::Observation Environment::getCurrentObservation() const {
    return createObservation();
}

void Environment::setAgent(std::shared_ptr<agent::Agent> agent) {
    agent_ = agent;
}

void Environment::setWorld(std::shared_ptr<sim::World> world) {
    world_ = world;
}

void Environment::setDrone(std::shared_ptr<sim::Drone> drone) {
    drone_ = drone;
}

float Environment::getAverageEpisodeLength() const {
    if (episode_history_.empty()) return 0.0f;
    
    float total = 0.0f;
    for (const auto& episode : episode_history_) {
        total += episode.steps;
    }
    
    return total / episode_history_.size();
}

float Environment::getSuccessRate() const {
    if (episode_history_.empty()) return 0.0f;
    
    int successes = 0;
    for (const auto& episode : episode_history_) {
        if (episode.success) successes++;
    }
    
    return static_cast<float>(successes) / episode_history_.size();
}

float Environment::calculateReward() const {
    if (!world_ || !drone_) return 0.0f;
    
    float reward = 0.0f;
    
    // Goal reward
    reward += getGoalReward();
    
    // Collision penalty
    reward += getCollisionPenalty();
    
    // Progress reward
    reward += getProgressReward();
    
    // Time penalty
    reward += getTimePenalty();
    
    // Safety margin penalty
    reward += getSafetyMarginPenalty();
    
    return reward;
}

bool Environment::checkEpisodeTermination() const {
    return isDone();
}

void Environment::updatePathTrace() {
    if (drone_) {
        path_trace_.push_back(drone_->getState().position);
    }
}

agent::Observation Environment::createObservation() const {
    agent::Observation obs;
    
    if (drone_ && world_) {
        // Create observation based on learning track
        if (agent_ && agent_->getConfig().use_vision) {
            obs.image = drone_->getEgoView(*world_, 84);
        } else {
            // Create occupancy grid for Track A
            cv::Mat grid = drone_->getOccupancyGrid(*world_, 21);
            obs.grid.resize(21 * 21);
            for (int i = 0; i < 21; ++i) {
                for (int j = 0; j < 21; ++j) {
                    obs.grid[i * 21 + j] = grid.at<uchar>(i, j) / 255.0f;
                }
            }
        }
        
        obs.heading = drone_->getState().heading;
        obs.distance_to_goal = drone_->getDistanceToGoal(world_->getGoalPosition());
        
        // Calculate goal direction
        cv::Point2f goal_dir = world_->getGoalPosition() - drone_->getState().position;
        obs.goal_direction = atan2(goal_dir.y, goal_dir.x);
    }
    
    return obs;
}

float Environment::getGoalReward() const {
    if (drone_ && world_ && drone_->hasReachedGoal(world_->getGoalPosition())) {
        return config_.goal_reward;
    }
    return 0.0f;
}

float Environment::getCollisionPenalty() const {
    if (world_ && drone_ && world_->checkCollision(drone_->getState().position, 10.0f)) {
        return config_.collision_penalty;
    }
    return 0.0f;
}

float Environment::getProgressReward() const {
    if (drone_ && world_) {
        float progress = drone_->getProgressToGoal(episode_start_, episode_goal_);
        return progress * config_.progress_reward;
    }
    return 0.0f;
}

float Environment::getTimePenalty() const {
    return config_.time_penalty;
}

float Environment::getSafetyMarginPenalty() const {
    // TODO: Implement safety margin penalty
    return 0.0f;
}

} // namespace bridge
