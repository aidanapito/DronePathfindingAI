#include "bridge/Environment.h"
#include "agent/QLearningAgent.h"

namespace bridge {

Environment::Environment(const EnvironmentConfig& config) 
    : config_(config), current_step_(0), cumulative_reward_(0.0f),
      use_pathfinding_(true), pathfinding_algorithm_("astar"), current_waypoint_index_(0) {
}

void Environment::reset() {
    current_step_ = 0;
    cumulative_reward_ = 0.0f;
    path_trace_.clear();
    
    if (world_ && drone_) {
        episode_start_ = drone_->getState().position;
        episode_goal_ = world_->getGoalPosition();
        
        // Compute optimal path if pathfinding is enabled
        if (use_pathfinding_) {
            computeOptimalPath();
        }
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
    agent::Action action = agent->selectAction(obs, *drone_);
    
    // TODO: Execute action and get next observation
    agent::Observation next_obs = obs; // Placeholder
    
    // Calculate reward
    float reward = getCurrentReward();
    cumulative_reward_ += reward;
    
    // Update agent policy
    agent->updatePolicy(obs, action, reward, next_obs, isDone());
    
    // Update path trace
    updatePathTrace();
    
    // Update waypoint progress if using pathfinding
    if (use_pathfinding_) {
        updateWaypointProgress();
        
        // Pass optimal path to QLearningAgent if it's a QLearningAgent
        if (auto q_agent = std::dynamic_pointer_cast<agent::QLearningAgent>(agent)) {
            q_agent->setOptimalPath(optimal_path_);
        }
    }
    
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
        obs.position = drone_->getState().position;
        
        // Calculate goal direction
        cv::Point2f goal_dir = world_->getGoalPosition() - drone_->getState().position;
        obs.goal_direction = atan2(goal_dir.y, goal_dir.x);
    }
    
    return obs;
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
    
    // Path following reward (when pathfinding is enabled)
    if (use_pathfinding_ && !optimal_path_.empty()) {
        float path_distance = getDistanceToPath();
        float path_reward = -path_distance * 0.01f; // Small penalty for deviating from path
        reward += path_reward;
        
        // Bonus for reaching waypoints
        if (current_waypoint_index_ > 0 && current_waypoint_index_ < optimal_path_.size()) {
            reward += 5.0f; // Small bonus for waypoint progress
        }
    }
    
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
        obs.position = drone_->getState().position;
        
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

// Pathfinding integration methods
std::vector<cv::Point2f> Environment::getOptimalPath() const {
    return optimal_path_;
}

void Environment::computeOptimalPath() {
    if (!world_ || !drone_) return;
    
    optimal_path_.clear();
    current_waypoint_index_ = 0;
    
    cv::Point2f start = drone_->getState().position;
    cv::Point2f goal = world_->getGoalPosition();
    
    if (pathfinding_algorithm_ == "astar") {
        optimal_path_ = world_->findPathAStar(start, goal, 10.0f);
    } else if (pathfinding_algorithm_ == "floodfill") {
        optimal_path_ = world_->findPathFloodFill(start, goal, 10.0f);
    }
    
    // If no path found, create a direct line to goal
    if (optimal_path_.empty()) {
        optimal_path_.push_back(start);
        optimal_path_.push_back(goal);
    }
    
    std::cout << "Computed optimal path with " << optimal_path_.size() << " waypoints" << std::endl;
}

float Environment::getDistanceToPath() const {
    if (optimal_path_.empty() || !drone_) return 0.0f;
    
    cv::Point2f current_pos = drone_->getState().position;
    float min_distance = std::numeric_limits<float>::max();
    
    // Find minimum distance to any point on the path
    for (const auto& waypoint : optimal_path_) {
        float distance = cv::norm(current_pos - waypoint);
        min_distance = std::min(min_distance, distance);
    }
    
    return min_distance;
}

cv::Point2f Environment::getNextWaypoint() const {
    if (optimal_path_.empty() || current_waypoint_index_ >= optimal_path_.size()) {
        return world_ ? world_->getGoalPosition() : cv::Point2f(0, 0);
    }
    return optimal_path_[current_waypoint_index_];
}

bool Environment::hasReachedWaypoint() const {
    if (optimal_path_.empty() || current_waypoint_index_ >= optimal_path_.size()) {
        return false;
    }
    
    cv::Point2f current_pos = drone_->getState().position;
    cv::Point2f waypoint = optimal_path_[current_waypoint_index_];
    float distance = cv::norm(current_pos - waypoint);
    
    return distance < 20.0f; // 20 pixel threshold
}

void Environment::updateWaypointProgress() {
    if (hasReachedWaypoint()) {
        current_waypoint_index_++;
        std::cout << "Reached waypoint " << current_waypoint_index_ - 1 
                  << " of " << optimal_path_.size() << std::endl;
    }
}

} // namespace bridge
