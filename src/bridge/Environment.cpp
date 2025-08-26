#include "bridge/Environment.h"
#include "agent/QLearningAgent.h"

namespace bridge {

Environment::Environment(const EnvironmentConfig& config) 
    : config_(config), current_step_(0), episode_count_(0), cumulative_reward_(0.0f),
      use_pathfinding_(true), pathfinding_algorithm_("astar"), current_waypoint_index_(0) {
    // Initialize member variables
    current_step_ = 0;
    episode_count_ = 0;
    cumulative_reward_ = 0.0f;
    path_trace_.clear();
    episode_start_ = cv::Point2f(0, 0);
    episode_goal_ = cv::Point2f(0, 0);
    use_pathfinding_ = false;
    pathfinding_algorithm_ = "astar";
    current_waypoint_index_ = 0;
    last_action_ = agent::Action::IDLE;
}

void Environment::reset() {
    current_step_ = 0;
    cumulative_reward_ = 0.0f;
    path_trace_.clear();
    
    // Reset action statistics
    action_stats_ = ActionStats{};
    
    // Reset last action
    last_action_ = agent::Action::IDLE;
    
    if (world_ && drone_) {
        // Reset drone to start position
        cv::Point2f start_pos = world_->getStartPosition();
        float start_heading = 0.0f;
        
        sim::DroneState start_state;
        start_state.position = start_pos;
        start_state.heading = start_heading;
        start_state.velocity = 0.0f;
        start_state.angular_velocity = 0.0f;
        
        drone_->setState(start_state);
        
        episode_start_ = start_pos;
        episode_goal_ = world_->getGoalPosition();
        
        // Reset agent state
        if (agent_) {
            agent_->reset();
        }
        
        // Compute optimal path if pathfinding is enabled
        if (use_pathfinding_) {
            computeOptimalPath();
            current_waypoint_index_ = 0; // Reset waypoint progress
        }
        
        // Clear path trace and add starting position
        path_trace_.clear();
        path_trace_.push_back(start_pos);
        
        std::cout << "Environment reset: Drone at (" << start_pos.x << ", " << start_pos.y 
                  << "), Goal at (" << episode_goal_.x << ", " << episode_goal_.y << ")" << std::endl;
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
    episode_count_++; // Increment episode counter
    return result;
}

void Environment::step(std::shared_ptr<agent::Agent> agent) {
    if (!agent || !world_ || !drone_) return;
    
    // Get current observation
    agent::Observation obs = getCurrentObservation();
    
    // Agent selects action
    agent::Action action = agent->selectAction(obs, *drone_);
    
    // Store last action for reward shaping
    last_action_ = action;
    
    // Log selected action for debugging
    std::string action_name;
    switch (action) {
        case agent::Action::THROTTLE_FORWARD: action_name = "FORWARD"; break;
        case agent::Action::YAW_LEFT: action_name = "TURN_LEFT"; break;
        case agent::Action::YAW_RIGHT: action_name = "TURN_RIGHT"; break;
        case agent::Action::IDLE: action_name = "IDLE"; break;
        default: action_name = "UNKNOWN"; break;
    }
    
    if (current_step_ % config_.action_log_frequency == 0) { // Log based on configuration
        std::cout << "Step " << current_step_ << ": Agent selected " << action_name 
                  << " (Distance to goal: " << obs.distance_to_goal << ")" << std::endl;
    }
    
    // Execute action on drone
    float throttle = 0.0f;
    float yaw_rate = 0.0f;
    
    // Track action statistics
    action_stats_.total_actions++;
    
    switch (action) {
        case agent::Action::THROTTLE_FORWARD:
            throttle = 1.0f * config_.throttle_scale;  // Apply throttle scaling
            yaw_rate = 0.0f;
            action_stats_.forward_actions++;
            break;
        case agent::Action::YAW_LEFT:
            throttle = 0.0f;
            yaw_rate = -1.0f * config_.yaw_rate_scale; // Apply yaw rate scaling
            action_stats_.left_turn_actions++;
            break;
        case agent::Action::YAW_RIGHT:
            throttle = 0.0f;
            yaw_rate = 1.0f * config_.yaw_rate_scale;  // Apply yaw rate scaling
            action_stats_.right_turn_actions++;
            break;
        case agent::Action::IDLE:
        default:
            throttle = 0.0f;
            yaw_rate = 0.0f;
            action_stats_.idle_actions++;
            break;
    }
    
    // Safety check: Check if action would cause collision (if enabled)
    if (config_.enable_safety_checks && drone_->wouldCollide(throttle, yaw_rate, config_.time_step)) {
        // Emergency stop - override action with safe values
        throttle = 0.0f;
        yaw_rate = 0.0f;
        action_stats_.blocked_actions++;
        std::cout << "Safety: Action blocked due to potential collision" << std::endl;
    }
    
    // Constraint check: Ensure action is within drone limits (if enabled)
    if (config_.enable_safety_checks && !drone_->isWithinConstraints(throttle, yaw_rate)) {
        // Scale down action to fit within constraints
        float scale_factor = 0.5f;
        throttle *= scale_factor;
        yaw_rate *= scale_factor;
        action_stats_.scaled_actions++;
        std::cout << "Safety: Action scaled down to fit constraints" << std::endl;
    }
    
    // Update drone physics
    drone_->update(config_.time_step, throttle, yaw_rate);
    
    // Check for emergency stop after action
    if (drone_->isEmergencyStop()) {
        action_stats_.emergency_stops++;
        std::cout << "Safety: Emergency stop activated" << std::endl;
    }
    
    // Get next observation after action execution
    agent::Observation next_obs = getCurrentObservation();
    
    // Track progress and success
    float previous_distance = obs.distance_to_goal;
    float current_distance = next_obs.distance_to_goal;
    float progress = previous_distance - current_distance;
    
    if (progress > 5.0f) { // Significant progress threshold
        std::cout << "Progress: Moved " << progress << " units closer to goal!" << std::endl;
    }
    
    // Check if goal was reached
    if (drone_->hasReachedGoal(world_->getGoalPosition())) {
        std::cout << "SUCCESS: Goal reached in " << current_step_ << " steps!" << std::endl;
    }
    
    // Check for collisions and boundary violations
    cv::Point2f current_pos = drone_->getState().position;
    if (world_->checkCollision(current_pos, 10.0f)) {
        std::cout << "WARNING: Collision detected at position (" 
                  << current_pos.x << ", " << current_pos.y << ")" << std::endl;
    }
    
    if (!world_->isInBounds(current_pos)) {
        std::cout << "WARNING: Drone out of bounds at position (" 
                  << current_pos.x << ", " << current_pos.y << ")" << std::endl;
    }
    
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
    
    // Log episode statistics periodically
    if (current_step_ % 50 == 0) {
        std::cout << "Episode Progress: Step " << current_step_ 
                  << ", Reward: " << cumulative_reward_
                  << ", Distance: " << next_obs.distance_to_goal
                  << ", Success Rate: " << (getSuccessRate() * 100.0f) << "%" << std::endl;
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
    
    // Enhanced goal-seeking rewards
    if (drone_) {
        cv::Point2f current_pos = drone_->getState().position;
        cv::Point2f goal_pos = world_->getGoalPosition();
        float current_distance = cv::norm(current_pos - goal_pos);
        float start_distance = cv::norm(episode_start_ - goal_pos);
        
        // 1. DIRECT DISTANCE REWARD: Reward for being closer to goal
        float distance_progress = start_distance - current_distance;
        if (distance_progress > 0) {
            // Exponential reward for getting closer (more reward for significant progress)
            float progress_ratio = distance_progress / start_distance;
            reward += progress_ratio * 10.0f; // Scale up progress rewards
        }
        
        // 2. DIRECTIONAL REWARD: Reward for moving toward goal
        if (path_trace_.size() >= 2) {
            cv::Point2f current_pos = path_trace_.back();
            cv::Point2f prev_pos = path_trace_[path_trace_.size() - 2];
            
            // Calculate movement vector
            cv::Point2f movement = current_pos - prev_pos;
            float movement_magnitude = cv::norm(movement);
            
            if (movement_magnitude > 0.1f) { // Only if actually moving
                // Calculate goal direction
                cv::Point2f goal_direction = goal_pos - prev_pos;
                float goal_distance = cv::norm(goal_direction);
                
                if (goal_distance > 0.1f) {
                    // Normalize vectors
                    goal_direction = goal_direction * (1.0f / goal_distance);
                    cv::Point2f normalized_movement = movement * (1.0f / movement_magnitude);
                    
                    // Dot product: 1.0 = moving directly toward goal, -1.0 = moving away
                    float alignment = goal_direction.dot(normalized_movement);
                    
                    // Reward for moving toward goal, penalty for moving away
                    if (alignment > 0.7f) {
                        reward += 2.0f; // Strong reward for moving toward goal
                    } else if (alignment > 0.3f) {
                        reward += 0.5f; // Small reward for somewhat toward goal
                    } else if (alignment < -0.5f) {
                        reward -= 1.0f; // Penalty for moving away from goal
                    }
                }
            }
        }
        
        // 3. PATH FOLLOWING REWARD (when pathfinding is enabled)
        if (use_pathfinding_ && !optimal_path_.empty()) {
            float path_distance = getDistanceToPath();
            
            // Exponential penalty for deviating from path (more penalty for larger deviations)
            if (path_distance > 0) {
                float path_penalty = -std::min(path_distance * 0.05f, 5.0f); // Cap penalty at -5
                reward += path_penalty;
            }
            
            // Enhanced waypoint rewards
            if (current_waypoint_index_ > 0 && current_waypoint_index_ < optimal_path_.size()) {
                // Bonus for reaching waypoints (scaled by progress)
                float waypoint_progress = static_cast<float>(current_waypoint_index_) / optimal_path_.size();
                reward += waypoint_progress * 10.0f; // More reward for later waypoints
                
                // Additional bonus for being close to next waypoint
                cv::Point2f next_waypoint = getNextWaypoint();
                float distance_to_waypoint = cv::norm(current_pos - next_waypoint);
                if (distance_to_waypoint < 30.0f) {
                    reward += (30.0f - distance_to_waypoint) * 0.1f; // Closer = more reward
                }
            }
        }
        
        // 4. EFFICIENCY REWARDS: Encourage smart movement patterns
        if (path_trace_.size() > 5) {
            // Reward for covering new ground (not retracing steps)
            cv::Point2f recent_center(0, 0);
            int lookback = std::min(5, static_cast<int>(path_trace_.size()));
            
            for (int i = path_trace_.size() - lookback; i < path_trace_.size(); ++i) {
                recent_center += path_trace_[i];
            }
            recent_center = recent_center * (1.0f / lookback);
            
            float area_covered = 0.0f;
            for (int i = path_trace_.size() - lookback; i < path_trace_.size(); ++i) {
                area_covered += cv::norm(path_trace_[i] - recent_center);
            }
            
            // If exploring new area, give reward; if oscillating, small penalty
            if (area_covered > 100.0f) {
                reward += 1.0f; // Reward for exploration
            } else if (area_covered < 30.0f) {
                reward -= 0.2f; // Small penalty for oscillation
            }
        }
        
        // 5. BOUNDARY REWARDS: Encourage staying in bounds
        if (world_->isInBounds(current_pos)) {
            reward += 0.2f; // Small bonus for staying in bounds
        } else {
            reward -= 2.0f; // Stronger penalty for going out of bounds
        }
        
        // 6. SPEED EFFICIENCY: Reward for moving at good speed toward goal
        if (path_trace_.size() >= 2) {
            cv::Point2f current_pos = path_trace_.back();
            cv::Point2f prev_pos = path_trace_[path_trace_.size() - 2];
            float step_distance = cv::norm(current_pos - prev_pos);
            
            // Reward for making meaningful progress (not just tiny movements)
            if (step_distance > 5.0f) {
                reward += 0.5f; // Reward for substantial movement
            }
        }
        
        // 7. GOAL PROXIMITY BONUS: Extra reward when getting very close to goal
        float goal_proximity = 1.0f - (current_distance / start_distance);
        if (goal_proximity > 0.8f) {
            reward += 5.0f; // Bonus for being very close to goal
        } else if (goal_proximity > 0.5f) {
            reward += 2.0f; // Bonus for being moderately close
        }
        
        // 8. ACTION-SPECIFIC REWARDS: Encourage efficient movement patterns
        if (path_trace_.size() >= 2) {
            cv::Point2f current_pos = path_trace_.back();
            cv::Point2f prev_pos = path_trace_[path_trace_.size() - 2];
            cv::Point2f goal_pos = world_->getGoalPosition();
            
            // Calculate if the last action moved us toward the goal
            float prev_distance = cv::norm(prev_pos - goal_pos);
            float current_distance = cv::norm(current_pos - goal_pos);
            bool moved_toward_goal = current_distance < prev_distance;
            
            // Reward/penalty based on action effectiveness
            switch (last_action_) {
                case agent::Action::THROTTLE_FORWARD:
                    if (moved_toward_goal) {
                        reward += 1.0f; // Reward for effective forward movement
                    } else {
                        reward -= 0.5f; // Small penalty for ineffective forward movement
                    }
                    break;
                    
                case agent::Action::YAW_LEFT:
                case agent::Action::YAW_RIGHT:
                    // Reward turning if it aligns us better with goal direction
                    if (path_trace_.size() >= 3) {
                        cv::Point2f two_back = path_trace_[path_trace_.size() - 3];
                        cv::Point2f old_direction = prev_pos - two_back;
                        cv::Point2f new_direction = current_pos - prev_pos;
                        
                        if (cv::norm(old_direction) > 0.1f && cv::norm(new_direction) > 0.1f) {
                            cv::Point2f goal_direction = goal_pos - prev_pos;
                            float old_alignment = old_direction.dot(goal_direction) / (cv::norm(old_direction) * cv::norm(goal_direction));
                            float new_alignment = new_direction.dot(goal_direction) / (cv::norm(new_direction) * cv::norm(goal_direction));
                            
                            if (new_alignment > old_alignment) {
                                reward += 0.5f; // Reward for turning toward goal
                            } else {
                                reward -= 0.3f; // Small penalty for turning away from goal
                            }
                        }
                    }
                    break;
                    
                case agent::Action::IDLE:
                    // Small penalty for doing nothing (encourage action)
                    reward -= 0.1f;
                    break;
            }
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
    if (!drone_ || !world_) return 0.0f;
    
    cv::Point2f current_pos = drone_->getState().position;
    cv::Point2f goal_pos = world_->getGoalPosition();
    float current_distance = cv::norm(current_pos - goal_pos);
    
    // Strong goal reward when very close
    if (current_distance < 20.0f) {
        return 500.0f; // Massive reward for reaching goal
    } else if (current_distance < 50.0f) {
        return 100.0f; // High reward for being very close
    } else if (current_distance < 100.0f) {
        return 50.0f;  // Good reward for being close
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
    if (!drone_ || !world_ || path_trace_.size() < 2) return 0.0f;
    
    cv::Point2f current_pos = drone_->getState().position;
    cv::Point2f goal_pos = world_->getGoalPosition();
    float current_distance = cv::norm(current_pos - goal_pos);
    
    // Calculate progress from start
    float start_distance = cv::norm(episode_start_ - goal_pos);
    float progress = start_distance - current_distance;
    
    // Enhanced progress reward
    if (progress > 0) {
        // Making progress toward goal
        return progress * 0.5f; // Reward proportional to progress
    } else {
        // Moving away from goal
        return progress * 0.2f; // Smaller penalty for moving away
    }
}

float Environment::getDirectionalReward() const {
    if (!drone_ || !world_ || path_trace_.size() < 2) return 0.0f;
    
    cv::Point2f current_pos = path_trace_.back();
    cv::Point2f prev_pos = path_trace_[path_trace_.size() - 2];
    cv::Point2f goal_pos = world_->getGoalPosition();
    
    cv::Point2f movement = current_pos - prev_pos;
    float movement_magnitude = cv::norm(movement);
    
    if (movement_magnitude > 0.1f) {
        cv::Point2f goal_direction = goal_pos - prev_pos;
        float goal_distance = cv::norm(goal_direction);
        
        if (goal_distance > 0.1f) {
            goal_direction = goal_direction * (1.0f / goal_distance);
            cv::Point2f normalized_movement = movement * (1.0f / movement_magnitude);
            float alignment = goal_direction.dot(normalized_movement);
            
            // Enhanced directional rewards
            if (alignment > 0.9f) {
                return 5.0f;  // Excellent alignment
            } else if (alignment > 0.7f) {
                return 3.0f;  // Good alignment
            } else if (alignment > 0.3f) {
                return 1.0f;  // Moderate alignment
            } else if (alignment < -0.5f) {
                return -2.0f; // Moving away from goal
            }
        }
    }
    
    return 0.0f;
}

float Environment::getPathFollowingReward() const {
    if (!use_pathfinding_ || optimal_path_.empty()) return 0.0f;
    
    float path_distance = getDistanceToPath();
    float reward = 0.0f;
    
    // Reward for staying close to optimal path
    if (path_distance < 20.0f) {
        reward += 2.0f; // Close to path
    } else if (path_distance < 50.0f) {
        reward += 1.0f; // Moderately close to path
    } else {
        reward -= std::min(path_distance * 0.1f, 10.0f); // Penalty for straying
    }
    
    // Additional reward for waypoint progress
    if (current_waypoint_index_ > 0 && current_waypoint_index_ < optimal_path_.size()) {
        float waypoint_progress = static_cast<float>(current_waypoint_index_) / optimal_path_.size();
        reward += waypoint_progress * 20.0f; // Significant reward for waypoint progress
    }
    
    return reward;
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

void Environment::printActionStats() const {
    std::cout << "\n=== Action Execution Statistics ===" << std::endl;
    std::cout << "Total Actions: " << action_stats_.total_actions << std::endl;
    std::cout << "Forward Actions: " << action_stats_.forward_actions 
              << " (" << (action_stats_.total_actions > 0 ? (100.0f * action_stats_.forward_actions / action_stats_.total_actions) : 0.0f) << "%)" << std::endl;
    std::cout << "Left Turn Actions: " << action_stats_.left_turn_actions 
              << " (" << (action_stats_.total_actions > 0 ? (100.0f * action_stats_.left_turn_actions / action_stats_.total_actions) : 0.0f) << "%)" << std::endl;
    std::cout << "Right Turn Actions: " << action_stats_.right_turn_actions 
              << " (" << (action_stats_.total_actions > 0 ? (100.0f * action_stats_.right_turn_actions / action_stats_.total_actions) : 0.0f) << "%)" << std::endl;
    std::cout << "Idle Actions: " << action_stats_.idle_actions 
              << " (" << (action_stats_.total_actions > 0 ? (100.0f * action_stats_.idle_actions / action_stats_.total_actions) : 0.0f) << "%)" << std::endl;
    std::cout << "Blocked Actions: " << action_stats_.blocked_actions 
              << " (" << (action_stats_.total_actions > 0 ? (100.0f * action_stats_.blocked_actions / action_stats_.total_actions) : 0.0f) << "%)" << std::endl;
    std::cout << "Scaled Actions: " << action_stats_.scaled_actions 
              << " (" << (action_stats_.total_actions > 0 ? (100.0f * action_stats_.scaled_actions / action_stats_.total_actions) : 0.0f) << "%)" << std::endl;
    std::cout << "Emergency Stops: " << action_stats_.emergency_stops << std::endl;
    std::cout << "=====================================" << std::endl;
}

Environment::RewardBreakdown Environment::getRewardBreakdown() const {
    RewardBreakdown breakdown;
    
    if (!world_ || !drone_) return breakdown;
    
    // Calculate each reward component
    breakdown.goal_reward = getGoalReward();
    breakdown.collision_penalty = getCollisionPenalty();
    breakdown.progress_reward = getProgressReward();
    breakdown.time_penalty = getTimePenalty();
    breakdown.safety_penalty = getSafetyMarginPenalty();
    
    // Calculate enhanced rewards (simplified version for breakdown)
    cv::Point2f current_pos = drone_->getState().position;
    cv::Point2f goal_pos = world_->getGoalPosition();
    float current_distance = cv::norm(current_pos - goal_pos);
    float start_distance = cv::norm(episode_start_ - goal_pos);
    
    // Directional reward
    if (path_trace_.size() >= 2) {
        cv::Point2f current_pos = path_trace_.back();
        cv::Point2f prev_pos = path_trace_[path_trace_.size() - 2];
        cv::Point2f movement = current_pos - prev_pos;
        float movement_magnitude = cv::norm(movement);
        
        if (movement_magnitude > 0.1f) {
            cv::Point2f goal_direction = goal_pos - prev_pos;
            float goal_distance = cv::norm(goal_direction);
            
            if (goal_distance > 0.1f) {
                goal_direction = goal_direction * (1.0f / goal_distance);
                cv::Point2f normalized_movement = movement * (1.0f / movement_magnitude);
                float alignment = goal_direction.dot(normalized_movement);
                
                if (alignment > 0.7f) {
                    breakdown.directional_reward = 2.0f;
                } else if (alignment > 0.3f) {
                    breakdown.directional_reward = 0.5f;
                } else if (alignment < -0.5f) {
                    breakdown.directional_reward = -1.0f;
                }
            }
        }
    }
    
    // Path following reward
    if (use_pathfinding_ && !optimal_path_.empty()) {
        float path_distance = getDistanceToPath();
        if (path_distance > 0) {
            breakdown.path_following_reward = -std::min(path_distance * 0.05f, 5.0f);
        }
        
        if (current_waypoint_index_ > 0 && current_waypoint_index_ < optimal_path_.size()) {
            float waypoint_progress = static_cast<float>(current_waypoint_index_) / optimal_path_.size();
            breakdown.path_following_reward += waypoint_progress * 10.0f;
        }
    }
    
    // Efficiency reward
    if (path_trace_.size() > 5) {
        cv::Point2f recent_center(0, 0);
        int lookback = std::min(5, static_cast<int>(path_trace_.size()));
        
        for (int i = path_trace_.size() - lookback; i < path_trace_.size(); ++i) {
            recent_center += path_trace_[i];
        }
        recent_center = recent_center * (1.0f / lookback);
        
        float area_covered = 0.0f;
        for (int i = path_trace_.size() - lookback; i < path_trace_.size(); ++i) {
            area_covered += cv::norm(path_trace_[i] - recent_center);
        }
        
        if (area_covered > 100.0f) {
            breakdown.efficiency_reward = 1.0f;
        } else if (area_covered < 30.0f) {
            breakdown.efficiency_reward = -0.2f;
        }
    }
    
    // Boundary reward
    if (world_->isInBounds(current_pos)) {
        breakdown.boundary_reward = 0.2f;
    } else {
        breakdown.boundary_reward = -2.0f;
    }
    
    // Speed reward
    if (path_trace_.size() >= 2) {
        cv::Point2f current_pos = path_trace_.back();
        cv::Point2f prev_pos = path_trace_[path_trace_.size() - 2];
        float step_distance = cv::norm(current_pos - prev_pos);
        
        if (step_distance > 5.0f) {
            breakdown.speed_reward = 0.5f;
        }
    }
    
    // Proximity reward
    float goal_proximity = 1.0f - (current_distance / start_distance);
    if (goal_proximity > 0.8f) {
        breakdown.proximity_reward = 5.0f;
    } else if (goal_proximity > 0.5f) {
        breakdown.proximity_reward = 2.0f;
    }
    
    // Action reward (simplified)
    if (path_trace_.size() >= 2) {
        cv::Point2f current_pos = path_trace_.back();
        cv::Point2f prev_pos = path_trace_[path_trace_.size() - 2];
        cv::Point2f goal_pos = world_->getGoalPosition();
        
        float prev_distance = cv::norm(prev_pos - goal_pos);
        float current_distance = cv::norm(current_pos - goal_pos);
        bool moved_toward_goal = current_distance < prev_distance;
        
        switch (last_action_) {
            case agent::Action::THROTTLE_FORWARD:
                breakdown.action_reward = moved_toward_goal ? 1.0f : -0.5f;
                break;
            case agent::Action::IDLE:
                breakdown.action_reward = -0.1f;
                break;
            default:
                breakdown.action_reward = 0.0f;
                break;
        }
    }
    
    // Calculate total
    breakdown.total_reward = breakdown.goal_reward + breakdown.collision_penalty + 
                             breakdown.progress_reward + breakdown.time_penalty + 
                             breakdown.safety_penalty + breakdown.directional_reward + 
                             breakdown.path_following_reward + breakdown.efficiency_reward + 
                             breakdown.boundary_reward + breakdown.speed_reward + 
                             breakdown.proximity_reward + breakdown.action_reward;
    
    return breakdown;
}

} // namespace bridge
