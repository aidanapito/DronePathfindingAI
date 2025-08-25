#include "agent/QLearningAgent.h"
#include <random>
#include <algorithm>
#include <cmath>

namespace agent {

QLearningAgent::QLearningAgent(const AgentConfig& config) 
    : Agent(config), alpha_(0.1f), gamma_(0.99f), epsilon_(0.1f),
      grid_resolution_(20), num_heading_buckets_(8),
      stuck_counter_(0), last_progress_step_(0), last_best_distance_(std::numeric_limits<float>::max()),
      is_exploring_(false), exploration_steps_(0), is_backtracking_(false), 
      is_panic_mode_(false), panic_counter_(0), is_goal_seeking_(false), wall_following_steps_(0) {
}

Action QLearningAgent::selectAction(const Observation& obs, const sim::Drone& drone) {
    QState state = discretizeState(obs);
    
    // Get valid actions early so they can be used throughout the method
    std::vector<Action> valid_actions = getValidActions(drone);
    if (valid_actions.empty()) {
        return Action::IDLE; // Fallback
    }
    
    // Check if we're stuck and need special handling
    float current_distance = obs.distance_to_goal;
    
    // Check if we need to enter panic mode
    if (stuck_counter_ > PANIC_THRESHOLD && !is_panic_mode_) {
        is_panic_mode_ = true;
        panic_counter_ = 0;
        is_exploring_ = false;
        is_backtracking_ = false;
        std::cout << "Agent entering PANIC MODE - extreme stuck situation!" << std::endl;
    }
    
    // Also check if we're in a persistent loop situation
    if (detectLoop() && stuck_counter_ > PANIC_THRESHOLD / 2 && !is_panic_mode_) {
        is_panic_mode_ = true;
        panic_counter_ = 0;
        is_exploring_ = false;
        is_backtracking_ = false;
        std::cout << "Agent entering PANIC MODE due to persistent loops!" << std::endl;
    }
    
    // Handle panic mode (highest priority - takes full control)
    if (is_panic_mode_) {
        Action action = selectPanicAction(state, valid_actions);
        panic_counter_++;
        
        if (panic_counter_ >= PANIC_DURATION) {
            is_panic_mode_ = false;
            panic_counter_ = 0;
            stuck_counter_ = 0;
            std::cout << "Panic mode ended, resetting stuck detection" << std::endl;
        }
        
        return action;
    }
    
    // Check if we should enter goal-seeking mode (when not in immediate danger)
    if (!is_panic_mode_ && !is_exploring_ && !is_backtracking_ && stuck_counter_ < STUCK_THRESHOLD / 3) {
        // If we're not in immediate danger, try to navigate toward the goal
        is_goal_seeking_ = true;
    }
    
    // Handle goal-seeking mode (higher priority than exploration, lower than panic)
    if (is_goal_seeking_ && !is_panic_mode_) {
        Action action = selectGoalSeekingAction(obs, valid_actions);
        
        // Exit goal-seeking mode if we're making good progress or if we hit obstacles
        if (current_distance < last_best_distance_ - PROGRESS_THRESHOLD) {
            is_goal_seeking_ = false;
            resetStuckDetection();
        } else if (stuck_counter_ > STUCK_THRESHOLD / 2) {
            is_goal_seeking_ = false;
        }
        
        return action;
    }
    
    // Only allow other modes if not in panic mode
    if (isStuck(current_distance)) {
        if (!is_exploring_ && !is_backtracking_ && !is_panic_mode_ && !is_goal_seeking_) {
            // Start exploration mode
            is_exploring_ = true;
            exploration_steps_ = 0;
            // Don't reset stuck_counter_ here - let it accumulate to trigger panic mode
            std::cout << "Agent is stuck, starting exploration mode" << std::endl;
        }
    }
    
    // Handle exploration mode
    if (is_exploring_) {
        Action action = selectExplorationAction(state, valid_actions);
        exploration_steps_++;
        
        if (exploration_steps_ >= EXPLORATION_DURATION) {
            is_exploring_ = false;
            exploration_steps_ = 0;
            std::cout << "Exploration mode ended" << std::endl;
        }
        
        return action;
    }
    
    // Handle backtracking mode
    if (is_backtracking_) {
        // Check if we should terminate backtracking
        if (shouldTerminateBacktracking(current_distance)) {
            is_backtracking_ = false;
            backtrack_path_.clear();
            std::cout << "Backtracking completed, returning to normal mode" << std::endl;
        } else {
            Action action = selectBacktrackAction(obs.position, obs.heading, valid_actions);
            return action;
        }
    }
    
    // Normal Q-learning action selection with valid action filtering
    if (epsilon_ > 0.0f && (static_cast<float>(rand()) / RAND_MAX) < epsilon_) {
        return epsilonGreedyAction(state, valid_actions);
    } else {
        return greedyAction(state, valid_actions);
    }
}

void QLearningAgent::updatePolicy(const Observation& obs, Action action, 
                                  float reward, const Observation& next_obs, bool done) {
    QState state = discretizeState(obs);
    QState next_state = discretizeState(next_obs);
    
    // Update path history
    updatePathHistory(obs.position, obs.heading, reward);
    
    // Update Q-values
    updateQValue(state, action, reward, next_state, done);
    
    // Check for progress and update stuck detection
    float current_distance = obs.distance_to_goal;
    float progress = calculateProgress(current_distance);
    
    if (progress > PROGRESS_THRESHOLD) {
        resetStuckDetection();
    } else {
        stuck_counter_++;
    }
    
    // Check if we need to start backtracking
    if (stuck_counter_ > STUCK_THRESHOLD && !is_backtracking_ && !is_exploring_ && !is_panic_mode_) {
        is_backtracking_ = true;
        std::cout << "Starting backtracking mode" << std::endl;
    }
}

void QLearningAgent::reset() {
    // Reset episode-specific state
    path_history_.clear();
    visited_cells_.clear();
    resetStuckDetection();
    is_exploring_ = false;
    exploration_steps_ = 0;
    is_backtracking_ = false;
    backtrack_path_.clear();
    is_panic_mode_ = false;
    panic_counter_ = 0;
    is_goal_seeking_ = false;
    wall_following_steps_ = 0;
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
    
    // Discretize grid position using actual drone position
    auto grid_pos = worldToGrid(obs.position);
    state.grid_x = grid_pos.first;
    state.grid_y = grid_pos.second;
    
    // Discretize heading
    state.heading_bucket = headingToBucket(obs.heading);
    
    return state;
}

Action QLearningAgent::epsilonGreedyAction(const QState& state, const std::vector<Action>& valid_actions) {
    // Random action for exploration
    if (valid_actions.empty()) {
        return Action::IDLE; // Fallback if no valid actions
    }
    return valid_actions[rand() % valid_actions.size()];
}

Action QLearningAgent::greedyAction(const QState& state, const std::vector<Action>& valid_actions) {
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

// Path memory and loop detection methods

void QLearningAgent::updatePathHistory(const cv::Point2f& position, float heading, float reward) {
    // Add current position to path history
    path_history_.emplace_back(position, heading, 0, reward);
    
    // Keep only recent history
    if (path_history_.size() > MAX_PATH_HISTORY) {
        path_history_.pop_front();
    }
    
    // Update visited cells
    auto grid_pos = worldToGrid(position);
    visited_cells_.insert(grid_pos);
}

bool QLearningAgent::detectLoop() const {
    if (path_history_.size() < LOOP_DETECTION_WINDOW) {
        return false;
    }
    
    // Check for repeated patterns in recent history - be more sensitive
    int recent_size = std::min(LOOP_DETECTION_WINDOW, static_cast<int>(path_history_.size()));
    
    for (int i = 0; i < recent_size - 1; ++i) {
        for (int j = i + 1; j < recent_size; ++j) {
            const auto& pos1 = path_history_[path_history_.size() - 1 - i].position;
            const auto& pos2 = path_history_[path_history_.size() - 1 - j].position;
            
            float distance = cv::norm(pos1 - pos2);
            // Reduce threshold to detect smaller loops
            if (distance < grid_resolution_ * 0.8f) {
                return true; // Potential loop detected
            }
        }
    }
    
    // Check for circular movement patterns - be more sensitive
    if (path_history_.size() >= LOOP_DETECTION_WINDOW) {
        cv::Point2f center(0, 0);
        float total_radius = 0.0f;
        
        // Calculate center and average radius of recent positions
        for (int i = path_history_.size() - LOOP_DETECTION_WINDOW; i < path_history_.size(); ++i) {
            center += path_history_[i].position;
        }
        center = center * (1.0f / LOOP_DETECTION_WINDOW);
        
        for (int i = path_history_.size() - LOOP_DETECTION_WINDOW; i < path_history_.size(); ++i) {
            total_radius += cv::norm(path_history_[i].position - center);
        }
        float avg_radius = total_radius / LOOP_DETECTION_WINDOW;
        
        // Check if positions form a roughly circular pattern - be more sensitive
        if (avg_radius > MIN_CIRCULAR_RADIUS * 0.8f && avg_radius < MAX_CIRCULAR_RADIUS * 1.2f) {
            // Check if the path is roughly circular by comparing start and end positions
            cv::Point2f start_pos = path_history_[path_history_.size() - LOOP_DETECTION_WINDOW].position;
            cv::Point2f end_pos = path_history_.back().position;
            float start_end_distance = cv::norm(start_pos - end_pos);
            
            // Reduce threshold to detect more circular patterns
            if (start_end_distance < avg_radius * CIRCULAR_MOVEMENT_THRESHOLD * 1.2f) {
                return true; // Circular movement detected
            }
        }
    }
    
    return false;
}

bool QLearningAgent::isStuck(float current_distance) const {
    // Check if we're making progress
    if (current_distance < last_best_distance_ - PROGRESS_THRESHOLD) {
        return false;
    }
    
    // Check if we've been stuck for too many steps
    if (stuck_counter_ > STUCK_THRESHOLD) {
        return true;
    }
    
    // Check for loops - this is the most important indicator
    if (detectLoop()) {
        return true;
    }
    
    // Check if we're oscillating in a small area - be more lenient
    if (path_history_.size() >= LOOP_DETECTION_WINDOW) {
        float total_area = 0.0f;
        cv::Point2f center(0, 0);
        
        // Calculate center of recent positions
        for (int i = path_history_.size() - LOOP_DETECTION_WINDOW; i < path_history_.size(); ++i) {
            center += path_history_[i].position;
        }
        center = center * (1.0f / LOOP_DETECTION_WINDOW);
        
        // Calculate area covered
        for (int i = path_history_.size() - LOOP_DETECTION_WINDOW; i < path_history_.size(); ++i) {
            float dist = cv::norm(path_history_[i].position - center);
            total_area += dist;
        }
        
        float avg_area = total_area / LOOP_DETECTION_WINDOW;
        // Be more lenient - only consider stuck if oscillating in very small area
        if (avg_area < grid_resolution_ * 1.5f) {
            return true; // Oscillating in very small area
        }
    }
    
    // Additional check: if we're very close to goal but not making progress, don't trigger stuck
    if (current_distance < 50.0f) {
        return false;
    }
    
    return false;
}

Action QLearningAgent::selectExplorationAction(const QState& state, const std::vector<Action>& valid_actions) {
    // Use more balanced exploration when stuck, avoiding circular patterns
    if (valid_actions.empty()) {
        return Action::IDLE; // Fallback if no valid actions
    }
    
    // Check if we're in a circular pattern and need to break out
    if (exploration_steps_ < EXPLORATION_DURATION / 3) {
        // First third: gentle exploration with reduced turning bias
        // Filter actions to only use valid ones
        std::vector<Action> gentle_actions;
        for (const auto& action : {Action::THROTTLE_FORWARD, Action::THROTTLE_FORWARD,
                                  Action::YAW_LEFT, Action::YAW_RIGHT, Action::IDLE}) {
            if (std::find(valid_actions.begin(), valid_actions.end(), action) != valid_actions.end()) {
                gentle_actions.push_back(action);
            }
        }
        
        if (!gentle_actions.empty()) {
            return gentle_actions[rand() % gentle_actions.size()];
        }
    } else if (exploration_steps_ < 2 * EXPLORATION_DURATION / 3) {
        // Middle third: more balanced exploration with goal-seeking behavior
        if (!path_history_.empty()) {
            cv::Point2f current_pos = path_history_.back().position;
            
            // Try to find actions that move toward the goal
            std::vector<Action> goal_seeking_actions;
            
            // Check if we can move forward toward the goal
            if (std::find(valid_actions.begin(), valid_actions.end(), Action::THROTTLE_FORWARD) != valid_actions.end()) {
                // Calculate if moving forward would get us closer to the goal
                cv::Point2f goal_pos = cv::Point2f(750.0f, 300.0f); // Approximate goal position
                float current_dist = cv::norm(current_pos - goal_pos);
                
                // Simulate forward movement
                cv::Point2f forward_pos = current_pos + cv::Point2f(
                    cos(path_history_.back().heading) * 20.0f,
                    sin(path_history_.back().heading) * 20.0f
                );
                float forward_dist = cv::norm(forward_pos - goal_pos);
                
                if (forward_dist < current_dist) {
                    goal_seeking_actions.push_back(Action::THROTTLE_FORWARD);
                    goal_seeking_actions.push_back(Action::THROTTLE_FORWARD); // Double weight
                }
            }
            
            // Check which directions lead to less visited areas
            std::vector<Action> preferred_actions;
            
            // Try to find unexplored directions with preference for forward movement
            for (int i = 0; i < 8; ++i) {
                float angle = i * M_PI / 4.0f;
                cv::Point2f test_pos = current_pos + cv::Point2f(
                    cos(angle) * grid_resolution_ * 2,
                    sin(angle) * grid_resolution_ * 2
                );
                
                auto test_grid = worldToGrid(test_pos);
                if (visited_cells_.find(test_grid) == visited_cells_.end()) {
                    // Unexplored area, prefer actions that move in this direction
                    if (std::abs(angle) < 0.5f && 
                        std::find(valid_actions.begin(), valid_actions.end(), Action::THROTTLE_FORWARD) != valid_actions.end()) {
                        preferred_actions.push_back(Action::THROTTLE_FORWARD);
                        preferred_actions.push_back(Action::THROTTLE_FORWARD); // Double weight for forward
                    } else if (angle > 0 && 
                               std::find(valid_actions.begin(), valid_actions.end(), Action::YAW_LEFT) != valid_actions.end()) {
                        preferred_actions.push_back(Action::YAW_LEFT);
                    } else if (angle < 0 && 
                               std::find(valid_actions.begin(), valid_actions.end(), Action::YAW_RIGHT) != valid_actions.end()) {
                        preferred_actions.push_back(Action::YAW_RIGHT);
                    }
                }
            }
            
            // Prioritize goal-seeking actions if available
            if (!goal_seeking_actions.empty()) {
                return goal_seeking_actions[rand() % goal_seeking_actions.size()];
            }
            
            if (!preferred_actions.empty()) {
                return preferred_actions[rand() % preferred_actions.size()];
            }
        }
        
        // Fallback to valid actions
        return valid_actions[rand() % valid_actions.size()];
    } else {
        // Final third: aggressive exploration to escape loops
        // Use a more random approach but still avoid pure circular patterns
        std::vector<Action> escape_actions;
        for (const auto& action : {Action::THROTTLE_FORWARD, Action::THROTTLE_FORWARD,
                                  Action::YAW_LEFT, Action::YAW_RIGHT, Action::IDLE}) {
            if (std::find(valid_actions.begin(), valid_actions.end(), action) != valid_actions.end()) {
                escape_actions.push_back(action);
            }
        }
        
        if (escape_actions.empty()) {
            return valid_actions[rand() % valid_actions.size()];
        }
        
        // Add some randomness to break out of patterns
        if (rand() % 100 < 30) { // 30% chance for random action
            return valid_actions[rand() % valid_actions.size()];
        }
        
        return escape_actions[rand() % escape_actions.size()];
    }
    
    // Final fallback
    return valid_actions[rand() % valid_actions.size()];
}

Action QLearningAgent::selectBacktrackAction(const cv::Point2f& current_pos, float current_heading, const std::vector<Action>& valid_actions) {
    if (backtrack_path_.empty()) {
        // Generate backtrack path if empty
        updateBacktrackPath(current_pos, current_heading);
    }
    
    if (backtrack_path_.empty()) {
        // Fallback to exploration if no backtrack path
        is_backtracking_ = false;
        return selectExplorationAction(QState{0, 0, 0}, valid_actions);
    }
    
    // Get target position from backtrack path
    cv::Point2f target = backtrack_path_.back();
    float target_angle = atan2(target.y - current_pos.y, target.x - current_pos.x);
    
    // Calculate angle difference
    float angle_diff = target_angle - current_heading;
    while (angle_diff > M_PI) angle_diff -= 2.0f * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2.0f * M_PI;
    
    // Select action based on angle difference, but only if it's valid
    if (std::abs(angle_diff) < 0.3f) {
        // Roughly facing target, move forward
        if (std::find(valid_actions.begin(), valid_actions.end(), Action::THROTTLE_FORWARD) != valid_actions.end()) {
            backtrack_path_.pop_back();
            return Action::THROTTLE_FORWARD;
        }
    } else if (angle_diff > 0) {
        if (std::find(valid_actions.begin(), valid_actions.end(), Action::YAW_LEFT) != valid_actions.end()) {
            return Action::YAW_LEFT;
        }
    } else {
        if (std::find(valid_actions.begin(), valid_actions.end(), Action::YAW_RIGHT) != valid_actions.end()) {
            return Action::YAW_RIGHT;
        }
    }
    
    // If preferred action is not valid, fall back to a valid action
    if (!valid_actions.empty()) {
        return valid_actions[rand() % valid_actions.size()];
    }
    
    return Action::IDLE;
}

void QLearningAgent::updateBacktrackPath(const cv::Point2f& position, float current_heading) {
    backtrack_path_.clear();
    
    // Create a path back to a previously visited position
    if (path_history_.size() > 1) {
        // Find a good backtracking target (not too recent, not too far)
        for (int i = path_history_.size() - 2; i >= 0; --i) {
            const auto& target = path_history_[i].position;
            float distance = cv::norm(position - target);
            
            if (distance > BACKTRACK_DISTANCE && distance < BACKTRACK_DISTANCE * 3) {
                // Create intermediate waypoints
                int num_waypoints = static_cast<int>(distance / (BACKTRACK_DISTANCE / 2));
                for (int j = 0; j < num_waypoints; ++j) {
                    float t = static_cast<float>(j) / num_waypoints;
                    cv::Point2f waypoint = position + t * (target - position);
                    backtrack_path_.insert(backtrack_path_.begin(), waypoint);
                }
                backtrack_path_.push_back(target);
                break;
            }
        }
        
        // If no good target found, try to find a position with better reward
        if (backtrack_path_.empty()) {
            float best_reward = -std::numeric_limits<float>::max();
            cv::Point2f best_target = position;
            
            for (int i = 0; i < path_history_.size(); ++i) {
                if (path_history_[i].reward > best_reward) {
                    best_reward = path_history_[i].reward;
                    best_target = path_history_[i].position;
                }
            }
            
            if (best_reward > -std::numeric_limits<float>::max()) {
                float distance = cv::norm(position - best_target);
                if (distance > BACKTRACK_DISTANCE / 2) {
                    int num_waypoints = std::max(1, static_cast<int>(distance / (BACKTRACK_DISTANCE / 2)));
                    for (int j = 0; j < num_waypoints; ++j) {
                        float t = static_cast<float>(j) / num_waypoints;
                        cv::Point2f waypoint = position + t * (best_target - position);
                        backtrack_path_.insert(backtrack_path_.begin(), waypoint);
                    }
                    backtrack_path_.push_back(best_target);
                }
            }
        }
    }
    
    if (backtrack_path_.empty()) {
        // Fallback: move in opposite direction of current heading
        float opposite_angle = atan2(-sin(current_heading), -cos(current_heading));
        cv::Point2f opposite_pos = position + cv::Point2f(
            cos(opposite_angle) * BACKTRACK_DISTANCE,
            sin(opposite_angle) * BACKTRACK_DISTANCE
        );
        backtrack_path_.push_back(opposite_pos);
    }
}

float QLearningAgent::calculateProgress(float current_distance) {
    float progress = last_best_distance_ - current_distance;
    if (progress > 0) {
        last_best_distance_ = current_distance;
        last_progress_step_ = 0;
    }
    return progress;
}

void QLearningAgent::resetStuckDetection() {
    stuck_counter_ = 0;
    last_progress_step_ = 0;
    is_exploring_ = false;
    exploration_steps_ = 0;
    is_backtracking_ = false;
    backtrack_path_.clear();
    is_panic_mode_ = false;
    panic_counter_ = 0;
    is_goal_seeking_ = false;
    wall_following_steps_ = 0;
}

bool QLearningAgent::shouldTerminateBacktracking(float current_distance) const {
    // Terminate if we've made significant progress
    if (current_distance < last_best_distance_ - PROGRESS_THRESHOLD * 2) {
        return true;
    }
    
    // Terminate if backtracking path is empty
    if (backtrack_path_.empty()) {
        return true;
    }
    
    // Terminate if we've been backtracking for too long without progress
    if (stuck_counter_ > STUCK_THRESHOLD * 2) {
        return true;
    }
    
    // Terminate if we're close to the backtracking target
    if (!path_history_.empty() && !backtrack_path_.empty()) {
        cv::Point2f current_pos = path_history_.back().position;
        cv::Point2f target = backtrack_path_.back();
        float distance_to_target = cv::norm(current_pos - target);
        
        if (distance_to_target < grid_resolution_) {
            return true;
        }
    }
    
    return false;
}

std::string QLearningAgent::getDebugInfo() const {
    std::string info = "QLearningAgent Debug Info:\n";
    info += "  Path History Size: " + std::to_string(path_history_.size()) + "\n";
    info += "  Visited Cells: " + std::to_string(visited_cells_.size()) + "\n";
    info += "  Stuck Counter: " + std::to_string(stuck_counter_) + "\n";
    info += "  Last Best Distance: " + std::to_string(last_best_distance_) + "\n";
    info += "  Is Exploring: " + std::string(is_exploring_ ? "Yes" : "No") + "\n";
    info += "  Exploration Steps: " + std::to_string(exploration_steps_) + "\n";
    info += "  Is Backtracking: " + std::string(is_backtracking_ ? "Yes" : "No") + "\n";
    info += "  Backtrack Path Size: " + std::to_string(backtrack_path_.size()) + "\n";
    info += "  Is Panic Mode: " + std::string(is_panic_mode_ ? "Yes" : "No") + "\n";
    info += "  Panic Counter: " + std::to_string(panic_counter_) + "\n";
    info += "  Is Goal Seeking: " + std::string(is_goal_seeking_ ? "Yes" : "No") + "\n";
    info += "  Q-Table Size: " + std::to_string(q_table_.size()) + "\n";
    info += "  Average Q-Value: " + std::to_string(getAverageQValue()) + "\n";
    
    // Add stuck detection analysis
    if (path_history_.size() >= LOOP_DETECTION_WINDOW) {
        info += "  Loop Detection: " + std::string(detectLoop() ? "Loop Detected" : "No Loop") + "\n";
        
        // Calculate area coverage
        if (path_history_.size() >= LOOP_DETECTION_WINDOW) {
            cv::Point2f center(0, 0);
            for (int i = path_history_.size() - LOOP_DETECTION_WINDOW; i < path_history_.size(); ++i) {
                center += path_history_[i].position;
            }
            center = center * (1.0f / LOOP_DETECTION_WINDOW);
            
            float total_area = 0.0f;
            for (int i = path_history_.size() - LOOP_DETECTION_WINDOW; i < path_history_.size(); ++i) {
                total_area += cv::norm(path_history_[i].position - center);
            }
            float avg_area = total_area / LOOP_DETECTION_WINDOW;
            info += "  Recent Area Coverage: " + std::to_string(avg_area) + "\n";
        }
    }
    
    return info;
}

Action QLearningAgent::selectPanicAction(const QState& state, const std::vector<Action>& valid_actions) {
    if (valid_actions.empty()) {
        return Action::IDLE;
    }
    
    // In panic mode, use completely random actions to break out of any patterns
    // This is the most aggressive escape mechanism
    Action action = valid_actions[rand() % valid_actions.size()];
    
    // Occasionally force a specific action type to break patterns
    if (panic_counter_ % 10 == 0) {
        // Every 10 steps, try to force a different action type
        if (std::find(valid_actions.begin(), valid_actions.end(), Action::THROTTLE_FORWARD) != valid_actions.end()) {
            action = Action::THROTTLE_FORWARD;
        } else if (std::find(valid_actions.begin(), valid_actions.end(), Action::YAW_LEFT) != valid_actions.end()) {
            action = Action::YAW_LEFT;
        } else if (std::find(valid_actions.begin(), valid_actions.end(), Action::YAW_RIGHT) != valid_actions.end()) {
            action = Action::YAW_RIGHT;
        }
    }
    
    return action;
}

Action QLearningAgent::selectGoalSeekingAction(const Observation& obs, const std::vector<Action>& valid_actions) {
    if (valid_actions.empty()) {
        return Action::IDLE;
    }
    
    // Get current position and goal direction
    cv::Point2f current_pos = obs.position;
    float current_heading = obs.heading;
    float goal_direction = obs.goal_direction;
    
    // Calculate the angle difference between current heading and goal direction
    float angle_diff = goal_direction - current_heading;
    
    // Normalize angle difference to [-π, π]
    while (angle_diff > M_PI) angle_diff -= 2.0f * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2.0f * M_PI;
    
    // Check if we're near a wall (close to map boundaries)
    bool near_left_wall = current_pos.x < 50.0f;
    bool near_right_wall = current_pos.x > 750.0f;
    bool near_top_wall = current_pos.y < 50.0f;
    bool near_bottom_wall = current_pos.y > 550.0f;
    
    // If we're near a wall, prioritize wall-following behavior
    if (near_left_wall || near_right_wall || near_top_wall || near_bottom_wall) {
        wall_following_steps_++;
        
        // If we've been wall-following for too long without progress, use aggressive escape
        if (wall_following_steps_ > 30) { // 30 steps = about 0.5 seconds
            std::cout << "Goal-seeking: Wall-following timeout, using aggressive escape" << std::endl;
            
            // Aggressive escape: try to turn away from the wall and move forward
            if (near_left_wall && std::find(valid_actions.begin(), valid_actions.end(), Action::YAW_RIGHT) != valid_actions.end()) {
                return Action::YAW_RIGHT; // Turn away from left wall
            } else if (near_right_wall && std::find(valid_actions.begin(), valid_actions.end(), Action::YAW_LEFT) != valid_actions.end()) {
                return Action::YAW_LEFT; // Turn away from right wall
            } else if (near_top_wall && std::find(valid_actions.begin(), valid_actions.end(), Action::YAW_RIGHT) != valid_actions.end()) {
                return Action::YAW_RIGHT; // Turn away from top wall
            } else if (near_bottom_wall && std::find(valid_actions.begin(), valid_actions.end(), Action::YAW_LEFT) != valid_actions.end()) {
                return Action::YAW_LEFT; // Turn away from bottom wall
            }
            
            // If turning away doesn't work, try to move forward
            if (std::find(valid_actions.begin(), valid_actions.end(), Action::THROTTLE_FORWARD) != valid_actions.end()) {
                return Action::THROTTLE_FORWARD;
            }
        }
        
        // Debug output for wall-following
        if (near_left_wall) std::cout << "Goal-seeking: Near LEFT wall, using wall-following" << std::endl;
        else if (near_right_wall) std::cout << "Goal-seeking: Near RIGHT wall, using wall-following" << std::endl;
        else if (near_top_wall) std::cout << "Goal-seeking: Near TOP wall, using wall-following" << std::endl;
        else if (near_bottom_wall) std::cout << "Goal-seeking: Near BOTTOM wall, using wall-following" << std::endl;
        
        // Check if we're stuck in a corner (near multiple walls)
        bool in_corner = (near_left_wall && near_top_wall) || (near_left_wall && near_bottom_wall) ||
                        (near_right_wall && near_top_wall) || (near_right_wall && near_bottom_wall);
        
        if (in_corner) {
            std::cout << "Goal-seeking: In CORNER, using aggressive escape strategy" << std::endl;
            
            // Corner escape: try to move diagonally away from the corner
            if (near_left_wall && near_top_wall) {
                // Top-left corner - try to move down-right
                if (std::find(valid_actions.begin(), valid_actions.end(), Action::YAW_RIGHT) != valid_actions.end()) {
                    return Action::YAW_RIGHT; // Turn right to face down-right
                }
            } else if (near_left_wall && near_bottom_wall) {
                // Bottom-left corner - try to move up-right
                if (std::find(valid_actions.begin(), valid_actions.end(), Action::YAW_LEFT) != valid_actions.end()) {
                    return Action::YAW_LEFT; // Turn left to face up-right
                }
            } else if (near_right_wall && near_top_wall) {
                // Top-right corner - try to move down-left
                if (std::find(valid_actions.begin(), valid_actions.end(), Action::YAW_LEFT) != valid_actions.end()) {
                    return Action::YAW_LEFT; // Turn left to face down-left
                }
            } else if (near_right_wall && near_bottom_wall) {
                // Bottom-right corner - try to move up-left
                if (std::find(valid_actions.begin(), valid_actions.end(), Action::YAW_RIGHT) != valid_actions.end()) {
                    return Action::YAW_RIGHT; // Turn right to face up-left
                }
            }
            
            // If corner escape didn't work, try to move forward in any available direction
            if (std::find(valid_actions.begin(), valid_actions.end(), Action::THROTTLE_FORWARD) != valid_actions.end()) {
                return Action::THROTTLE_FORWARD;
            }
        }
        
        // Wall-following strategy: try to move parallel to the wall
        if (near_left_wall) {
            // Near left wall - try to move right (positive x direction)
            if (std::abs(current_heading - 0.0f) < 0.5f && 
                std::find(valid_actions.begin(), valid_actions.end(), Action::THROTTLE_FORWARD) != valid_actions.end()) {
                return Action::THROTTLE_FORWARD; // Move right
            } else if (current_heading > 0.5f && 
                       std::find(valid_actions.begin(), valid_actions.end(), Action::YAW_LEFT) != valid_actions.end()) {
                return Action::YAW_LEFT; // Turn left to face right
            } else if (current_heading < -0.5f && 
                       std::find(valid_actions.begin(), valid_actions.end(), Action::YAW_RIGHT) != valid_actions.end()) {
                return Action::YAW_RIGHT; // Turn right to face right
            }
        } else if (near_right_wall) {
            // Near right wall - try to move left (negative x direction)
            if (std::abs(current_heading - M_PI) < 0.5f && 
                std::find(valid_actions.begin(), valid_actions.end(), Action::THROTTLE_FORWARD) != valid_actions.end()) {
                return Action::THROTTLE_FORWARD; // Move left
            } else if (current_heading > 0.5f && 
                       std::find(valid_actions.begin(), valid_actions.end(), Action::YAW_RIGHT) != valid_actions.end()) {
                return Action::YAW_RIGHT; // Turn right to face left
            } else if (current_heading < -0.5f && 
                       std::find(valid_actions.begin(), valid_actions.end(), Action::YAW_LEFT) != valid_actions.end()) {
                return Action::YAW_LEFT; // Turn left to face left
            }
        } else if (near_top_wall) {
            // Near top wall - try to move down (positive y direction)
            if (std::abs(current_heading - M_PI/2.0f) < 0.5f && 
                std::find(valid_actions.begin(), valid_actions.end(), Action::THROTTLE_FORWARD) != valid_actions.end()) {
                return Action::THROTTLE_FORWARD; // Move down
            } else if (current_heading > M_PI/2.0f + 0.5f && 
                       std::find(valid_actions.begin(), valid_actions.end(), Action::YAW_LEFT) != valid_actions.end()) {
                return Action::YAW_LEFT; // Turn left to face down
            } else if (current_heading < M_PI/2.0f - 0.5f && 
                       std::find(valid_actions.begin(), valid_actions.end(), Action::YAW_RIGHT) != valid_actions.end()) {
                return Action::YAW_RIGHT; // Turn right to face down
            }
        } else if (near_bottom_wall) {
            // Near bottom wall - try to move up (negative y direction)
            if (std::abs(current_heading - (-M_PI/2.0f)) < 0.5f && 
                std::find(valid_actions.begin(), valid_actions.end(), Action::THROTTLE_FORWARD) != valid_actions.end()) {
                return Action::THROTTLE_FORWARD; // Move up
            } else if (current_heading > -M_PI/2.0f + 0.5f && 
                       std::find(valid_actions.begin(), valid_actions.end(), Action::YAW_RIGHT) != valid_actions.end()) {
                return Action::YAW_RIGHT; // Turn right to face up
            } else if (current_heading < -M_PI/2.0f - 0.5f && 
                       std::find(valid_actions.begin(), valid_actions.end(), Action::YAW_LEFT) != valid_actions.end()) {
                return Action::YAW_LEFT; // Turn left to face up
            }
        }
        
        // If wall-following didn't work, try to turn away from the wall
        if (near_left_wall && std::find(valid_actions.begin(), valid_actions.end(), Action::YAW_RIGHT) != valid_actions.end()) {
            return Action::YAW_RIGHT; // Turn away from left wall
        } else if (near_right_wall && std::find(valid_actions.begin(), valid_actions.end(), Action::YAW_LEFT) != valid_actions.end()) {
            return Action::YAW_LEFT; // Turn away from right wall
        } else if (near_top_wall && std::find(valid_actions.begin(), valid_actions.end(), Action::YAW_RIGHT) != valid_actions.end()) {
            return Action::YAW_RIGHT; // Turn away from top wall
        } else if (near_bottom_wall && std::find(valid_actions.begin(), valid_actions.end(), Action::YAW_LEFT) != valid_actions.end()) {
            return Action::YAW_LEFT; // Turn away from bottom wall
        }
    }
    
    // If we're not near walls, use normal goal-seeking behavior
    if (!near_left_wall && !near_right_wall && !near_top_wall && !near_bottom_wall) {
        wall_following_steps_ = 0; // Reset wall-following counter when not near walls
    }
    
    // If we're roughly facing the goal (within 30 degrees), try to move forward
    if (std::abs(angle_diff) < 0.5f && 
        std::find(valid_actions.begin(), valid_actions.end(), Action::THROTTLE_FORWARD) != valid_actions.end()) {
        return Action::THROTTLE_FORWARD;
    }
    
    // If we need to turn to face the goal, choose the appropriate turn action
    if (angle_diff > 0.1f) { // Need to turn left
        if (std::find(valid_actions.begin(), valid_actions.end(), Action::YAW_LEFT) != valid_actions.end()) {
            return Action::YAW_LEFT;
        }
    } else if (angle_diff < -0.1f) { // Need to turn right
        if (std::find(valid_actions.begin(), valid_actions.end(), Action::YAW_RIGHT) != valid_actions.end()) {
            return Action::YAW_RIGHT;
        }
    }
    
    // If we can't turn in the preferred direction, try the other direction
    if (angle_diff > 0.1f) { // Wanted to turn left but couldn't
        if (std::find(valid_actions.begin(), valid_actions.end(), Action::YAW_RIGHT) != valid_actions.end()) {
            return Action::YAW_RIGHT;
        }
    } else if (angle_diff < -0.1f) { // Wanted to turn right but couldn't
        if (std::find(valid_actions.begin(), valid_actions.end(), Action::YAW_LEFT) != valid_actions.end()) {
            return Action::YAW_LEFT;
        }
    }
    
    // If we can move forward, do so (even if not perfectly aligned)
    if (std::find(valid_actions.begin(), valid_actions.end(), Action::THROTTLE_FORWARD) != valid_actions.end()) {
        return Action::THROTTLE_FORWARD;
    }
    
    // Fallback to any valid action
    return valid_actions[rand() % valid_actions.size()];
}

} // namespace agent
