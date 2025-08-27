#include "bridge/Environment.h"
#include "agent/QLearningAgent.h"

namespace bridge {

Environment::Environment(const EnvironmentConfig& config) 
    : config_(config), current_step_(0), episode_count_(0), cumulative_reward_(0.0f),
      use_pathfinding_(true), pathfinding_algorithm_("astar"), current_waypoint_index_(0), depth_(400) {
    // Initialize member variables
    current_step_ = 0;
    episode_count_ = 0;
    cumulative_reward_ = 0.0f;
    path_trace_3d_.clear();
    path_trace_2d_.clear();
    episode_start_ = cv::Point3f(0, 0, 0);
    episode_goal_ = cv::Point3f(0, 0, 0);
    use_pathfinding_ = false;
    pathfinding_algorithm_ = "astar";
    current_waypoint_index_ = 0;
    last_action_ = agent::Action::IDLE;
}

void Environment::reset() {
    current_step_ = 0;
    cumulative_reward_ = 0.0f;
    path_trace_3d_.clear();
    path_trace_2d_.clear();
    
    // Reset action statistics
    action_stats_ = ActionStats{};
    
    // Reset last action
    last_action_ = agent::Action::IDLE;
    
    if (world_ && drone_) {
        // Reset drone to start position
        cv::Point3f start_pos = world_->getStartPosition();
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
            if (is3DPathfindingEnabled()) {
                computeOptimalPath3D();
            } else {
                computeOptimalPath();
            }
            current_waypoint_index_ = 0; // Reset waypoint progress
        }
        
        // Clear path trace and add starting position
        path_trace_3d_.clear();
        path_trace_3d_.push_back(start_pos);
        
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
    result.path_trace = path_trace_3d_;
    
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
    
    // Update 3D action statistics if in 3D mode
    if (is3DMode()) {
        update3DActionStats(action);
    }
    
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
    if (config_.enable_safety_checks && drone_->wouldCollide(throttle, yaw_rate, 0.0f, 0.0f, 0.0f, config_.time_step)) {
        // Emergency stop - override action with safe values
        throttle = 0.0f;
        yaw_rate = 0.0f;
        action_stats_.blocked_actions++;
        std::cout << "Safety: Action blocked due to potential collision" << std::endl;
    }
    
    // Constraint check: Ensure action is within drone limits (if enabled)
    if (config_.enable_safety_checks && !drone_->isWithinConstraints(throttle, yaw_rate, 0.0f, 0.0f, 0.0f)) {
        // Scale down action to fit within constraints
        float scale_factor = 0.5f;
        throttle *= scale_factor;
        yaw_rate *= scale_factor;
        action_stats_.scaled_actions++;
        std::cout << "Safety: Action scaled down to fit constraints" << std::endl;
    }
    
    // Update drone physics with full 3D controls
    float pitch_rate = 0.0f;  // No pitch change for now
    float roll_rate = 0.0f;   // No roll change for now
    float vertical_thrust = 0.0f; // No vertical movement for now
    
    drone_->update(config_.time_step, throttle, yaw_rate, pitch_rate, roll_rate, vertical_thrust);
    
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
    cv::Point3f current_pos_3d = drone_->getState().position;
    cv::Point2f current_pos_2d(current_pos_3d.x, current_pos_3d.y); // Convert to 2D for backward compatibility
    if (world_->checkCollision(current_pos_2d, 10.0f)) {
        std::cout << "WARNING: Collision detected at position (" 
                  << current_pos_2d.x << ", " << current_pos_2d.y << ")" << std::endl;
    }
    
    if (!world_->isInBounds(current_pos_2d)) {
        std::cout << "WARNING: Drone out of bounds at position (" 
                  << current_pos_2d.x << ", " << current_pos_2d.y << ")" << std::endl;
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
            if (is3DPathfindingEnabled()) {
                // Convert 3D path to 2D for backward compatibility
                std::vector<cv::Point2f> path_2d;
                for (const auto& point_3d : optimal_path_3d_) {
                    path_2d.push_back(cv::Point2f(point_3d.x, point_3d.y));
                }
                q_agent->setOptimalPath(path_2d);
            } else {
                q_agent->setOptimalPath(optimal_path_2d_);
            }
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
        cv::Point3f goal_pos_3d = world_->getGoalPosition();
        cv::Point2f goal_pos_2d(goal_pos_3d.x, goal_pos_3d.y); // Convert to 2D for backward compatibility
        cv::Point2f goal_dir = goal_pos_2d - cv::Point2f(obs.position.x, obs.position.y);
        obs.goal_direction = atan2(goal_dir.y, goal_dir.x);
    }
    
    return obs;
}

void Environment::setAgent(std::shared_ptr<agent::Agent> agent) {
    agent_ = agent;
}

void Environment::setWorld(std::shared_ptr<sim::World> world) {
    world_ = world;
    if (world_) {
        auto size = world_->getSize();
        depth_ = size.depth;
    }
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

float Environment::getAverageAltitudeError() const {
    if (episode_history_.empty()) return 0.0f;
    
    float total_error = 0.0f;
    int valid_episodes = 0;
    
    for (const auto& episode : episode_history_) {
        if (episode.altitude_error > 0) {
            total_error += episode.altitude_error;
            valid_episodes++;
        }
    }
    
    return valid_episodes > 0 ? total_error / valid_episodes : 0.0f;
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
    
    // 3D specific rewards
    if (is3DMode()) {
        reward += getAltitudeReward();
        reward += getVerticalProgressReward();
        reward += getClearanceReward();
    }
    
    // Enhanced goal-seeking rewards
    if (drone_) {
        cv::Point3f current_pos = drone_->getState().position;
        cv::Point3f goal_pos = world_->getGoalPosition();
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
        if (path_trace_3d_.size() >= 2) {
            cv::Point3f current_pos = path_trace_3d_.back();
            cv::Point3f prev_pos = path_trace_3d_[path_trace_3d_.size() - 2];
            cv::Point3f goal_pos = world_->getGoalPosition();
            
            // Calculate movement vector
            cv::Point3f movement = current_pos - prev_pos;
            float movement_magnitude = cv::norm(movement);
            
            if (movement_magnitude > 0.1f) { // Only if actually moving
                // Calculate goal direction
                cv::Point3f goal_direction = goal_pos - prev_pos;
                float goal_distance = cv::norm(goal_direction);
                
                if (goal_distance > 0.1f) {
                    // Normalize vectors
                    goal_direction = goal_direction * (1.0f / goal_distance);
                    cv::Point3f normalized_movement = movement * (1.0f / movement_magnitude);
                    
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
        if (use_pathfinding_) {
            float path_distance;
            if (is3DPathfindingEnabled() && !optimal_path_3d_.empty()) {
                path_distance = getDistanceToPath3D();
            } else if (!optimal_path_2d_.empty()) {
                path_distance = getDistanceToPath();
            } else {
                path_distance = 0.0f;
            }
            
            // Exponential penalty for deviating from path (more penalty for larger deviations)
            if (path_distance > 0) {
                float path_penalty = -std::min(path_distance * 0.05f, 5.0f); // Cap penalty at -5
                reward += path_penalty;
            }
            
            // Enhanced waypoint rewards
            if (current_waypoint_index_ > 0) {
                int path_size = is3DPathfindingEnabled() ? optimal_path_3d_.size() : optimal_path_2d_.size();
                if (current_waypoint_index_ < path_size) {
                    // Bonus for reaching waypoints (scaled by progress)
                    float waypoint_progress = static_cast<float>(current_waypoint_index_) / path_size;
                    reward += waypoint_progress * 10.0f; // More reward for later waypoints
                    
                    // Additional bonus for being close to next waypoint
                    cv::Point2f next_waypoint;
                    if (is3DPathfindingEnabled()) {
                        cv::Point3f next_3d = getNextWaypoint3D();
                        next_waypoint = cv::Point2f(next_3d.x, next_3d.y);
                    } else {
                        next_waypoint = getNextWaypoint();
                    }
                    
                    cv::Point2f current_pos_2d(current_pos.x, current_pos.y); // Convert to 2D for backward compatibility
                    float distance_to_waypoint = cv::norm(current_pos_2d - next_waypoint);
                    if (distance_to_waypoint < 30.0f) {
                        reward += (30.0f - distance_to_waypoint) * 0.1f; // Closer = more reward
                    }
                }
            }
        }
        
        // 4. EFFICIENCY REWARDS: Encourage smart movement patterns
        if (path_trace_3d_.size() > 5) {
            // Reward for covering new ground (not retracing steps)
            cv::Point3f recent_center(0, 0, 0);
            int lookback = std::min(5, static_cast<int>(path_trace_3d_.size()));
            
            for (int i = path_trace_3d_.size() - lookback; i < path_trace_3d_.size(); ++i) {
                recent_center += path_trace_3d_[i];
            }
            recent_center = recent_center * (1.0f / lookback);
            
            float area_covered = 0.0f;
            for (int i = path_trace_3d_.size() - lookback; i < path_trace_3d_.size(); ++i) {
                area_covered += cv::norm(path_trace_3d_[i] - recent_center);
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
        if (path_trace_3d_.size() >= 2) {
            cv::Point3f current_pos = path_trace_3d_.back();
            cv::Point3f prev_pos = path_trace_3d_[path_trace_3d_.size() - 2];
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
        if (path_trace_3d_.size() >= 2) {
            cv::Point3f current_pos = path_trace_3d_.back();
            cv::Point3f prev_pos = path_trace_3d_[path_trace_3d_.size() - 2];
            cv::Point3f goal_pos = world_->getGoalPosition();
            
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
                    if (path_trace_3d_.size() >= 3) {
                        cv::Point3f two_back = path_trace_3d_[path_trace_3d_.size() - 3];
                        cv::Point3f old_direction = prev_pos - two_back;
                        cv::Point3f new_direction = current_pos - prev_pos;
                        
                        if (cv::norm(old_direction) > 0.1f && cv::norm(new_direction) > 0.1f) {
                            cv::Point3f goal_direction = goal_pos - prev_pos;
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
        path_trace_3d_.push_back(drone_->getState().position);
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
        cv::Point3f goal_pos_3d = world_->getGoalPosition();
        cv::Point2f goal_pos_2d(goal_pos_3d.x, goal_pos_3d.y); // Convert to 2D for backward compatibility
        cv::Point2f goal_dir = goal_pos_2d - cv::Point2f(obs.position.x, obs.position.y);
        obs.goal_direction = atan2(goal_dir.y, goal_dir.x);
    }
    
    return obs;
}

float Environment::getGoalReward() const {
    if (!drone_ || !world_) return 0.0f;
    
    cv::Point3f current_pos = drone_->getState().position;
    cv::Point3f goal_pos = world_->getGoalPosition();
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
    if (!drone_ || !world_ || path_trace_3d_.size() < 2) return 0.0f;
    
    cv::Point3f current_pos = drone_->getState().position;
    cv::Point3f goal_pos = world_->getGoalPosition();
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

float Environment::getTimePenalty() const {
    return config_.time_penalty;
}

float Environment::getSafetyMarginPenalty() const {
    // TODO: Implement safety margin penalty
    return 0.0f;
}

float Environment::getAltitudeReward() const {
    if (!drone_ || !world_) return 0.0f;
    
    cv::Point3f current_pos = drone_->getState().position;
    cv::Point3f goal_pos = world_->getGoalPosition();
    
    // Reward for maintaining good altitude relative to goal
    float altitude_diff = std::abs(current_pos.z - goal_pos.z);
    if (altitude_diff < 20.0f) {
        return config_.altitude_reward; // Reward for being at similar altitude
    } else if (altitude_diff > 100.0f) {
        return config_.altitude_penalty; // Penalty for large altitude difference
    }
    
    return 0.0f;
}

float Environment::getVerticalProgressReward() const {
    if (!drone_ || !world_ || path_trace_3d_.size() < 2) return 0.0f;
    
    cv::Point3f current_pos = drone_->getState().position;
    cv::Point3f goal_pos = world_->getGoalPosition();
    cv::Point3f prev_pos = path_trace_3d_[path_trace_3d_.size() - 2];
    
    // Calculate vertical progress toward goal
    float prev_altitude_diff = std::abs(prev_pos.z - goal_pos.z);
    float current_altitude_diff = std::abs(current_pos.z - goal_pos.z);
    float vertical_progress = prev_altitude_diff - current_altitude_diff;
    
    if (vertical_progress > 0) {
        return vertical_progress * 0.5f; // Reward for moving toward goal altitude
    }
    
    return 0.0f;
}

float Environment::getClearanceReward() const {
    if (!drone_ || !world_) return 0.0f;
    
    cv::Point3f current_pos = drone_->getState().position;
    float clearance_reward = 0.0f;
    
    // Reward for maintaining safe distance from ground and ceiling
    float ground_distance = current_pos.z - 0.0f;
    float ceiling_distance = depth_ - current_pos.z;
    
    if (ground_distance < 20.0f) {
        clearance_reward -= (20.0f - ground_distance) * 0.1f; // Penalty for being too close to ground
    }
    
    if (ceiling_distance < 20.0f) {
        clearance_reward -= (20.0f - ceiling_distance) * 0.1f; // Penalty for being too close to ceiling
    }
    
    // Reward for being in the middle altitude range
    float optimal_altitude = depth_ / 2.0f;
    float altitude_deviation = std::abs(current_pos.z - optimal_altitude);
    if (altitude_deviation < 50.0f) {
        clearance_reward += 0.5f; // Small reward for being in safe altitude range
    }
    
    return clearance_reward;
}

// Pathfinding integration methods
std::vector<cv::Point2f> Environment::getOptimalPath() const {
    return optimal_path_2d_;
}

std::vector<cv::Point3f> Environment::getOptimalPath3D() const {
    return optimal_path_3d_;
}

void Environment::computeOptimalPath() {
    if (!world_ || !drone_) return;
    
    optimal_path_2d_.clear();
    current_waypoint_index_ = 0;
    
    cv::Point3f start_3d = drone_->getState().position;
    cv::Point2f start(start_3d.x, start_3d.y); // Convert 3D to 2D for backward compatibility
    cv::Point3f goal_3d = world_->getGoalPosition();
    cv::Point2f goal(goal_3d.x, goal_3d.y); // Convert 3D to 2D for backward compatibility
    
    if (pathfinding_algorithm_ == "astar") {
        optimal_path_2d_ = world_->findPathAStar(start, goal, 10.0f);
    } else if (pathfinding_algorithm_ == "floodfill") {
        optimal_path_2d_ = world_->findPathFloodFill(start, goal, 10.0f);
    }
    
    // If no path found, create a direct line to goal
    if (optimal_path_2d_.empty()) {
        optimal_path_2d_.push_back(start);
        optimal_path_2d_.push_back(goal);
    }
    
    std::cout << "Computed optimal path with " << optimal_path_2d_.size() << " waypoints" << std::endl;
}

void Environment::computeOptimalPath3D() {
    if (!world_ || !drone_) return;
    
    optimal_path_3d_.clear();
    current_waypoint_index_ = 0;
    
    cv::Point3f start_3d = drone_->getState().position;
    cv::Point3f goal_3d = world_->getGoalPosition();
    
    if (pathfinding_algorithm_ == "astar") {
        optimal_path_3d_ = world_->findPathAStar3D(start_3d, goal_3d, 10.0f);
    } else if (pathfinding_algorithm_ == "floodfill") {
        optimal_path_3d_ = world_->findPathFloodFill3D(start_3d, goal_3d, 10.0f);
    }
    
    // If no path found, create a direct line to goal
    if (optimal_path_3d_.empty()) {
        optimal_path_3d_.push_back(start_3d);
        optimal_path_3d_.push_back(goal_3d);
    }
    
    std::cout << "Computed 3D optimal path with " << optimal_path_3d_.size() << " waypoints" << std::endl;
}

float Environment::getDistanceToPath() const {
    if (optimal_path_2d_.empty() || !drone_) return 0.0f;
    
    cv::Point3f current_pos_3d = drone_->getState().position;
    cv::Point2f current_pos_2d(current_pos_3d.x, current_pos_3d.y); // Convert to 2D for backward compatibility
    float min_distance = std::numeric_limits<float>::max();
    
    // Find minimum distance to any point on the path
    for (const auto& waypoint : optimal_path_2d_) {
        float distance = cv::norm(current_pos_2d - waypoint);
        min_distance = std::min(min_distance, distance);
    }
    
    return min_distance;
}

float Environment::getDistanceToPath3D() const {
    if (optimal_path_3d_.empty() || !drone_) return 0.0f;
    
    cv::Point3f current_pos_3d = drone_->getState().position;
    float min_distance = std::numeric_limits<float>::max();
    
    // Find minimum distance to any point on the 3D path
    for (const auto& waypoint : optimal_path_3d_) {
        float distance = cv::norm(current_pos_3d - waypoint);
        min_distance = std::min(min_distance, distance);
    }
    
    return min_distance;
}

cv::Point3f Environment::getNextWaypoint3D() const {
    if (optimal_path_3d_.empty() || current_waypoint_index_ >= optimal_path_3d_.size()) {
        return world_ ? world_->getGoalPosition() : cv::Point3f(0, 0, 0);
    }
    return optimal_path_3d_[current_waypoint_index_];
}

cv::Point2f Environment::getNextWaypoint() const {
    if (optimal_path_2d_.empty() || current_waypoint_index_ >= optimal_path_2d_.size()) {
        cv::Point3f goal_3d = world_ ? world_->getGoalPosition() : cv::Point3f(0, 0, 0);
        return cv::Point2f(goal_3d.x, goal_3d.y); // Convert 3D to 2D for backward compatibility
    }
    return optimal_path_2d_[current_waypoint_index_];
}

bool Environment::hasReachedWaypoint() const {
    if (optimal_path_2d_.empty() || current_waypoint_index_ >= optimal_path_2d_.size()) {
        return false;
    }
    
    cv::Point3f current_pos_3d = drone_->getState().position;
    cv::Point2f current_pos_2d(current_pos_3d.x, current_pos_3d.y); // Convert to 2D for backward compatibility
    cv::Point2f waypoint = optimal_path_2d_[current_waypoint_index_];
    float distance = cv::norm(current_pos_2d - waypoint);
    
    return distance < 20.0f; // 20 pixel threshold
}

void Environment::updateWaypointProgress() {
    if (hasReachedWaypoint()) {
        current_waypoint_index_++;
        std::cout << "Reached waypoint " << current_waypoint_index_ - 1 
                  << " of " << optimal_path_2d_.size() << std::endl;
    }
}

void Environment::update3DActionStats(agent::Action action) {
    switch (action) {
        case agent::Action::PITCH_UP:
            action_stats_.pitch_up_actions++;
            break;
        case agent::Action::PITCH_DOWN:
            action_stats_.pitch_down_actions++;
            break;
        case agent::Action::ROLL_LEFT:
            action_stats_.roll_left_actions++;
            break;
        case agent::Action::ROLL_RIGHT:
            action_stats_.roll_right_actions++;
            break;
        case agent::Action::THRUST_UP:
            action_stats_.thrust_up_actions++;
            break;
        case agent::Action::THRUST_DOWN:
            action_stats_.thrust_down_actions++;
            break;
        default:
            break;
    }
    
    // Check if this is a combined 3D action
    if (action != agent::Action::IDLE && action != agent::Action::THROTTLE_FORWARD &&
        action != agent::Action::YAW_LEFT && action != agent::Action::YAW_RIGHT) {
        action_stats_.combined_3d_actions++;
    }
}

float Environment::calculate3DDistance(const cv::Point3f& p1, const cv::Point3f& p2) const {
    return cv::norm(p1 - p2);
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
    
    // 3D Action Statistics
    if (is3DMode()) {
        std::cout << "\n--- 3D Action Statistics ---" << std::endl;
        std::cout << "Pitch Up Actions: " << action_stats_.pitch_up_actions << std::endl;
        std::cout << "Pitch Down Actions: " << action_stats_.pitch_down_actions << std::endl;
        std::cout << "Roll Left Actions: " << action_stats_.roll_left_actions << std::endl;
        std::cout << "Roll Right Actions: " << action_stats_.roll_right_actions << std::endl;
        std::cout << "Thrust Up Actions: " << action_stats_.thrust_up_actions << std::endl;
        std::cout << "Thrust Down Actions: " << action_stats_.thrust_down_actions << std::endl;
        std::cout << "Combined 3D Actions: " << action_stats_.combined_3d_actions << std::endl;
    }
    
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
    cv::Point3f current_pos = drone_->getState().position;
    cv::Point3f goal_pos = world_->getGoalPosition();
    float current_distance = cv::norm(current_pos - goal_pos);
    float start_distance = cv::norm(episode_start_ - goal_pos);
    
    // Directional reward
    if (path_trace_3d_.size() >= 2) {
        cv::Point3f current_pos = path_trace_3d_.back();
        cv::Point3f prev_pos = path_trace_3d_[path_trace_3d_.size() - 2];
        cv::Point3f movement = current_pos - prev_pos;
        float movement_magnitude = cv::norm(movement);
        
        if (movement_magnitude > 0.1f) {
            cv::Point3f goal_direction = goal_pos - prev_pos;
            float goal_distance = cv::norm(goal_direction);
            
            if (goal_distance > 0.1f) {
                goal_direction = goal_direction * (1.0f / goal_distance);
                cv::Point3f normalized_movement = movement * (1.0f / movement_magnitude);
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
    if (use_pathfinding_) {
        float path_distance;
        if (is3DPathfindingEnabled() && !optimal_path_3d_.empty()) {
            path_distance = getDistanceToPath3D();
        } else if (!optimal_path_2d_.empty()) {
            path_distance = getDistanceToPath();
        } else {
            path_distance = 0.0f;
        }
        
        if (path_distance > 0) {
            breakdown.path_following_reward = -std::min(path_distance * 0.05f, 5.0f);
        }
        
        if (current_waypoint_index_ > 0) {
            int path_size = is3DPathfindingEnabled() ? optimal_path_3d_.size() : optimal_path_2d_.size();
            if (current_waypoint_index_ < path_size) {
                float waypoint_progress = static_cast<float>(current_waypoint_index_) / path_size;
                breakdown.path_following_reward += waypoint_progress * 10.0f;
            }
        }
    }
    
    // Efficiency reward
    if (path_trace_3d_.size() > 5) {
        cv::Point3f recent_center(0, 0, 0);
        int lookback = std::min(5, static_cast<int>(path_trace_3d_.size()));
        
        for (int i = path_trace_3d_.size() - lookback; i < path_trace_3d_.size(); ++i) {
            recent_center += path_trace_3d_[i];
        }
        recent_center = recent_center * (1.0f / lookback);
        
        float area_covered = 0.0f;
        for (int i = path_trace_3d_.size() - lookback; i < path_trace_3d_.size(); ++i) {
            area_covered += cv::norm(path_trace_3d_[i] - recent_center);
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
    if (path_trace_3d_.size() >= 2) {
        cv::Point3f current_pos = path_trace_3d_.back();
        cv::Point3f prev_pos = path_trace_3d_[path_trace_3d_.size() - 2];
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
    if (path_trace_3d_.size() >= 2) {
        cv::Point3f current_pos = path_trace_3d_.back();
        cv::Point3f prev_pos = path_trace_3d_[path_trace_3d_.size() - 2];
        cv::Point3f goal_pos = world_->getGoalPosition();
        
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
    
    // 3D specific rewards
    if (is3DMode()) {
        breakdown.altitude_reward = getAltitudeReward();
        breakdown.altitude_penalty = 0.0f; // This is already included in altitude_reward
        breakdown.vertical_progress_reward = getVerticalProgressReward();
        breakdown.clearance_reward = getClearanceReward();
    }
    
    // Calculate total
    breakdown.total_reward = breakdown.goal_reward + breakdown.collision_penalty + 
                             breakdown.progress_reward + breakdown.time_penalty + 
                             breakdown.safety_penalty + breakdown.directional_reward + 
                             breakdown.path_following_reward + breakdown.efficiency_reward + 
                             breakdown.boundary_reward + breakdown.speed_reward + 
                             breakdown.proximity_reward + breakdown.action_reward +
                             breakdown.altitude_reward + breakdown.vertical_progress_reward + 
                             breakdown.clearance_reward;
    
    return breakdown;
}

} // namespace bridge
