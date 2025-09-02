#include "AI/PathfindingAI.h"
#include <iostream>
#include <cmath>

PathfindingAI::PathfindingAI() 
    : current_mode_(AIMode::MANUAL)
    , current_state_(AIState::IDLE)
    , target_position_(0.0f, 0.0f, 0.0f)
    , home_position_(0.0f, 0.0f, 0.0f)
    , current_waypoint_index_(0)
    , safety_margin_(DEFAULT_SAFETY_MARGIN)
    , pathfinding_aggressiveness_(DEFAULT_PATHFINDING_AGGRESSIVENESS)
    , target_reached_threshold_(DEFAULT_TARGET_REACHED_THRESHOLD)
    , waypoint_reached_threshold_(DEFAULT_WAYPOINT_REACHED_THRESHOLD) {
    
    path_planner_ = std::make_unique<PathPlanner>();
    obstacle_avoidance_ = std::make_unique<ObstacleAvoidance>();
}

DroneInput PathfindingAI::update(float delta_time, const DroneState& current_state, const World& world) {
    switch (current_mode_) {
        case AIMode::MANUAL:
            return generateManualInput();
            
        case AIMode::FOLLOW_PATH:
            return generatePathFollowingInput(current_state, world, delta_time);
            
        case AIMode::EXPLORE:
            return generateExplorationInput(current_state, world, delta_time);
            
        case AIMode::RETURN_HOME:
            return generateReturnHomeInput(current_state, delta_time);
            
        case AIMode::AVOID_OBSTACLES:
            return generateObstacleAvoidanceInput(current_state, world, delta_time);
            
        default:
            return generateManualInput();
    }
}

void PathfindingAI::setMode(AIMode mode) {
    if (current_mode_ != mode) {
        current_mode_ = mode;
        current_state_ = AIState::IDLE;
        current_waypoint_index_ = 0;
        
        // Clear path when switching modes
        if (mode != AIMode::FOLLOW_PATH) {
            clearPath();
        } else {
            // For FOLLOW_PATH mode, clear path and set to IDLE to trigger planning
            clearPath();
            current_state_ = AIState::IDLE;
        }
        
        std::cout << "AI Mode changed to: ";
        switch (mode) {
            case AIMode::MANUAL: std::cout << "MANUAL"; break;
            case AIMode::FOLLOW_PATH: std::cout << "FOLLOW_PATH"; break;
            case AIMode::EXPLORE: std::cout << "EXPLORE"; break;
            case AIMode::RETURN_HOME: std::cout << "RETURN_HOME"; break;
            case AIMode::AVOID_OBSTACLES: std::cout << "AVOID_OBSTACLES"; break;
        }
        std::cout << std::endl;
    }
}

void PathfindingAI::setTarget(float x, float y, float z) {
    target_position_ = glm::vec3(x, y, z);
    current_waypoint_index_ = 0;
    clearPath();
    
    std::cout << "🎯 Target set to: (" << x << ", " << y << ", " << z << ")" << std::endl;
    
    if (current_mode_ == AIMode::FOLLOW_PATH) {
        current_state_ = AIState::PLANNING_PATH;
    }
}

void PathfindingAI::setHomePosition(float x, float y, float z) {
    home_position_ = glm::vec3(x, y, z);
}

void PathfindingAI::clearPath() {
    current_path_.clear();
    current_waypoint_index_ = 0;
}

bool PathfindingAI::isPathValid() const {
    return !current_path_.empty() && current_waypoint_index_ < current_path_.size();
}

bool PathfindingAI::isTargetReached() const {
    // This method should be called with current drone state
    // For now, return false as we don't have access to current position here
    return false;
}

float PathfindingAI::getDistanceToTarget() const {
    // This method should be called with current drone state
    // For now, return 0 as we don't have access to current position here
    return 0.0f;
}

const std::vector<glm::vec3>& PathfindingAI::getCurrentPath() const {
    return current_path_;
}

glm::vec3 PathfindingAI::getCurrentTarget() const {
    return target_position_;
}

DroneInput PathfindingAI::generateManualInput() {
    return DroneInput{0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
}

DroneInput PathfindingAI::generatePathFollowingInput(const DroneState& current_state, const World& world, float delta_time) {
    // If we're planning or have no path, wait for path planning to complete
    if (current_state_ == AIState::PLANNING_PATH || current_state_ == AIState::IDLE) {
        std::cout << "⏳ Waiting for path planning to complete..." << std::endl;
        return generateManualInput(); // Return neutral input while planning
    }
    
    // If we have an error or no path after planning, try to replan
    if (current_path_.empty()) {
        if (current_state_ == AIState::ERROR) {
            std::cout << "🔄 Path planning failed, attempting to replan..." << std::endl;
            current_state_ = AIState::IDLE;
            return generateManualInput();
        } else {
            std::cout << "⚠️  No path available, attempting direct movement" << std::endl;
            // Fall back to direct movement only as last resort
            glm::vec3 drone_pos(current_state.x, current_state.y, current_state.z);
            glm::vec3 to_target = target_position_ - drone_pos;
            
            // Calculate distance to target
            float distance_to_target = glm::length(to_target);
            std::cout << "   Distance to target: " << distance_to_target << " units" << std::endl;
            std::cout << "   Target position: (" << target_position_.x << ", " << target_position_.y << ", " << target_position_.z << ")" << std::endl;
            std::cout << "   Drone position: (" << drone_pos.x << ", " << drone_pos.y << ", " << drone_pos.z << ")" << std::endl;
            std::cout << "   Direction vector: (" << to_target.x << ", " << to_target.y << ", " << to_target.z << ")" << std::endl;
            
            // If we're very close to target, just hover
            if (distance_to_target < 10.0f) {
                std::cout << "   Very close to target, hovering..." << std::endl;
                return generateManualInput(); // Hover in place
            }
            
            // Project onto drone's forward direction
            glm::vec3 drone_forward(sin(current_state.yaw), cos(current_state.yaw), 0.0f);
            float forward_velocity = glm::dot(to_target, drone_forward);
            std::cout << "   Drone forward: (" << drone_forward.x << ", " << drone_forward.y << ", " << drone_forward.z << ")" << std::endl;
            std::cout << "   Forward velocity: " << forward_velocity << std::endl;
            
            DroneInput input{0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
            
            // Limit forward thrust to prevent spiraling
            input.forward_thrust = glm::clamp(forward_velocity / MAX_VELOCITY, -0.5f, 0.5f);
            
            // More conservative vertical movement
            float vertical_diff = to_target.z;
            input.vertical_thrust = glm::clamp(vertical_diff / MAX_VELOCITY, -0.3f, 0.3f);
            
            // Yaw control
            glm::vec3 target_direction = glm::normalize(to_target);
            float target_yaw = atan2(target_direction.y, target_direction.x);
            float yaw_error = target_yaw - current_state.yaw;
            while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
            while (yaw_error < -M_PI) yaw_error += 2 * M_PI;
            
            input.yaw_rate = glm::clamp(yaw_error * TURNING_RATE, -0.5f, 0.5f);
            
            std::cout << "   Input - F:" << input.forward_thrust << " Y:" << input.yaw_rate << " V:" << input.vertical_thrust << std::endl;
            
            return input;
        }
    }
    
    // Check if we've successfully landed on the target
    if (hasLandedOnTarget(current_state, world)) {
        current_state_ = AIState::COMPLETED_MISSION;
        std::cout << "🎉 SUCCESS! Drone has landed on the target building!" << std::endl;
        return generateManualInput();
    }
    
    // Get current waypoint
    if (current_waypoint_index_ >= current_path_.size()) {
        current_state_ = AIState::COMPLETED_MISSION;
        return generateManualInput();
    }
    
    glm::vec3 current_waypoint = current_path_[current_waypoint_index_];
    
    // Check if we've reached the current waypoint
    if (isWaypointReached(current_state, current_waypoint)) {
        advanceToNextWaypoint();
        if (current_waypoint_index_ >= current_path_.size()) {
            current_state_ = AIState::COMPLETED_MISSION;
            return generateManualInput();
        }
        current_waypoint = current_path_[current_waypoint_index_];
    }
    
    // Calculate desired velocity towards waypoint
    glm::vec3 desired_velocity = calculateDesiredVelocity(current_state, current_waypoint);
    
    // Convert velocity to drone input
    DroneInput input{0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    
    // Calculate the desired direction relative to drone's current orientation
    glm::vec3 drone_pos(current_state.x, current_state.y, current_state.z);
    glm::vec3 to_waypoint = current_waypoint - drone_pos;
    
    std::cout << "🎯 Waypoint following:" << std::endl;
    std::cout << "   Drone pos: (" << drone_pos.x << ", " << drone_pos.y << ", " << drone_pos.z << ")" << std::endl;
    std::cout << "   Waypoint: (" << current_waypoint.x << ", " << current_waypoint.y << ", " << current_waypoint.z << ")" << std::endl;
    std::cout << "   To waypoint: (" << to_waypoint.x << ", " << to_waypoint.y << ", " << to_waypoint.z << ")" << std::endl;
    
    // Project the desired direction onto the drone's forward direction
    glm::vec3 drone_forward(sin(current_state.yaw), cos(current_state.yaw), 0.0f);
    float forward_velocity = glm::dot(to_waypoint, drone_forward);
    std::cout << "   Drone yaw: " << current_state.yaw << " radians (" << (current_state.yaw * 180.0f / M_PI) << " degrees)" << std::endl;
    std::cout << "   Drone forward: (" << drone_forward.x << ", " << drone_forward.y << ", " << drone_forward.z << ")" << std::endl;
    std::cout << "   Forward velocity: " << forward_velocity << std::endl;
    input.forward_thrust = glm::clamp(forward_velocity / MAX_VELOCITY, -1.0f, 1.0f);
    
    // Vertical thrust based on desired vertical velocity
    float vertical_velocity = desired_velocity.z; // Z is up in our coordinate system
    input.vertical_thrust = glm::clamp(vertical_velocity / MAX_VELOCITY, -1.0f, 1.0f);
    
    // Yaw control to face the waypoint
    glm::vec3 current_direction(sin(current_state.yaw), cos(current_state.yaw), 0.0f);
    glm::vec3 target_direction = glm::normalize(glm::vec3(current_waypoint.x - current_state.x, 
                                                         current_waypoint.y - current_state.y, 0.0f));
    
    // Calculate yaw error (target yaw - current yaw)
    float target_yaw = atan2(target_direction.y, target_direction.x);
    float yaw_error = target_yaw - current_state.yaw;
    
    // Normalize yaw error to [-π, π]
    while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2 * M_PI;
    
    input.yaw_rate = glm::clamp(yaw_error * TURNING_RATE, -1.0f, 1.0f);
    
    // Pitch control for vertical movement
    float pitch_error = atan2(desired_velocity.z, sqrt(desired_velocity.x * desired_velocity.x + desired_velocity.y * desired_velocity.y));
    input.pitch_rate = glm::clamp(pitch_error * TURNING_RATE, -1.0f, 1.0f);
    
    std::cout << "   Final input - F:" << input.forward_thrust << " Y:" << input.yaw_rate << " P:" << input.pitch_rate << " V:" << input.vertical_thrust << std::endl;
    
    return input;
}

DroneInput PathfindingAI::generateObstacleAvoidanceInput(const DroneState& current_state, const World& world, float delta_time) {
    // Calculate avoidance vector
    glm::vec3 current_direction(cos(current_state.yaw), sin(current_state.yaw), 0.0f);
    AvoidanceVector avoidance = obstacle_avoidance_->calculateAvoidanceVector(current_state, world, current_direction);
    
    if (!avoidance.is_active) {
        // If no obstacles detected, move forward slowly
        DroneInput input{0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        input.forward_thrust = 0.3f; // Slow forward movement
        return input;
    }
    
    // Convert avoidance direction to drone input
    DroneInput input{0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    
    // Project avoidance direction onto drone's forward direction
    glm::vec3 drone_forward(cos(current_state.yaw), sin(current_state.yaw), 0.0f);
    float forward_velocity = glm::dot(avoidance.direction, drone_forward) * avoidance.strength;
    
    // Forward thrust
    input.forward_thrust = glm::clamp(forward_velocity, -1.0f, 1.0f);
    
    // Vertical thrust
    input.vertical_thrust = glm::clamp(avoidance.direction.z * avoidance.strength, -1.0f, 1.0f);
    
    // Yaw control
    float target_yaw = atan2(avoidance.direction.y, avoidance.direction.x);
    float yaw_error = target_yaw - current_state.yaw;
    while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2 * M_PI;
    
    input.yaw_rate = glm::clamp(yaw_error * TURNING_RATE, -1.0f, 1.0f);
    
    return input;
}

DroneInput PathfindingAI::generateExplorationInput(const DroneState& current_state, const World& world, float delta_time) {
    // Simple exploration: move in a spiral pattern
    static float exploration_time = 0.0f;
    exploration_time += delta_time;
    
    // Generate exploration target
    float radius = 50.0f + 10.0f * sin(exploration_time * 0.1f);
    float angle = exploration_time * 0.5f;
    float height = 50.0f + 20.0f * sin(exploration_time * 0.2f);
    
    glm::vec3 exploration_target(
        home_position_.x + radius * cos(angle),
        home_position_.y + radius * sin(angle),
        home_position_.z + height
    );
    
    // Calculate direction to target
    glm::vec3 drone_pos(current_state.x, current_state.y, current_state.z);
    glm::vec3 to_target = exploration_target - drone_pos;
    
    // Project onto drone's forward direction
    glm::vec3 drone_forward(cos(current_state.yaw), sin(current_state.yaw), 0.0f);
    float forward_velocity = glm::dot(to_target, drone_forward);
    
    DroneInput input{0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    
    input.forward_thrust = glm::clamp(forward_velocity / MAX_VELOCITY, -1.0f, 1.0f);
    input.vertical_thrust = glm::clamp(to_target.z / MAX_VELOCITY, -1.0f, 1.0f);
    
    // Yaw control
    glm::vec3 target_direction = glm::normalize(to_target);
    float target_yaw = atan2(target_direction.y, target_direction.x);
    float yaw_error = target_yaw - current_state.yaw;
    while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2 * M_PI;
    
    input.yaw_rate = glm::clamp(yaw_error * TURNING_RATE, -1.0f, 1.0f);
    
    return input;
}

DroneInput PathfindingAI::generateReturnHomeInput(const DroneState& current_state, float delta_time) {
    // Calculate direction to home
    glm::vec3 drone_pos(current_state.x, current_state.y, current_state.z);
    glm::vec3 to_home = home_position_ - drone_pos;
    
    // Project onto drone's forward direction
    glm::vec3 drone_forward(cos(current_state.yaw), sin(current_state.yaw), 0.0f);
    float forward_velocity = glm::dot(to_home, drone_forward);
    
    DroneInput input{0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    
    input.forward_thrust = glm::clamp(forward_velocity / MAX_VELOCITY, -1.0f, 1.0f);
    input.vertical_thrust = glm::clamp(to_home.z / MAX_VELOCITY, -1.0f, 1.0f);
    
    // Yaw control
    glm::vec3 target_direction = glm::normalize(to_home);
    float target_yaw = atan2(target_direction.y, target_direction.x);
    float yaw_error = target_yaw - current_state.yaw;
    while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2 * M_PI;
    
    input.yaw_rate = glm::clamp(yaw_error * TURNING_RATE, -1.0f, 1.0f);
    
    return input;
}

void PathfindingAI::updatePath(const DroneState& current_state, const World& world) {
    if (current_mode_ != AIMode::FOLLOW_PATH) {
        return;
    }
    
    current_state_ = AIState::PLANNING_PATH;
    std::cout << "🛣️  Planning path to target..." << std::endl;
    std::cout << "   Drone at: (" << current_state.x << ", " << current_state.y << ", " << current_state.z << ")" << std::endl;
    std::cout << "   Target at: (" << target_position_.x << ", " << target_position_.y << ", " << target_position_.z << ")" << std::endl;
    
    // Check if target is reasonable
    float distance_to_target = glm::length(target_position_ - glm::vec3(current_state.x, current_state.y, current_state.z));
    std::cout << "   Distance to target: " << distance_to_target << " units" << std::endl;
    
    if (distance_to_target < 1.0f) {
        std::cout << "   Target is very close, no path planning needed" << std::endl;
        current_state_ = AIState::COMPLETED_MISSION;
        return;
    }
    
    glm::vec3 start(current_state.x, current_state.y, current_state.z);
    current_path_ = path_planner_->findPath(start, target_position_, world);
    
    if (!current_path_.empty()) {
        current_state_ = AIState::FOLLOWING_PATH;
        current_waypoint_index_ = 0;
        std::cout << "✅ Path planned with " << current_path_.size() << " waypoints" << std::endl;
        
        // Debug: print first few waypoints
        for (size_t i = 0; i < std::min(current_path_.size(), size_t(3)); ++i) {
            std::cout << "   Waypoint " << i << ": (" << current_path_[i].x << ", " << current_path_[i].y << ", " << current_path_[i].z << ")" << std::endl;
        }
    } else {
        current_state_ = AIState::ERROR;
        std::cout << "❌ Failed to plan path to target" << std::endl;
    }
}

bool PathfindingAI::isWaypointReached(const DroneState& current_state, const glm::vec3& waypoint) const {
    glm::vec3 current_pos(current_state.x, current_state.y, current_state.z);
    float distance = glm::length(waypoint - current_pos);
    return distance < waypoint_reached_threshold_;
}

void PathfindingAI::advanceToNextWaypoint() {
    current_waypoint_index_++;
    if (current_waypoint_index_ < current_path_.size()) {
        std::cout << "🎯 Reached waypoint " << (current_waypoint_index_ - 1) 
                  << ", moving to waypoint " << current_waypoint_index_ 
                  << " at (" << current_path_[current_waypoint_index_].x 
                  << ", " << current_path_[current_waypoint_index_].y 
                  << ", " << current_path_[current_waypoint_index_].z << ")" << std::endl;
    } else {
        std::cout << "🎯 Reached final waypoint, mission complete!" << std::endl;
    }
}

glm::vec3 PathfindingAI::calculateDesiredVelocity(const DroneState& current_state, const glm::vec3& target) const {
    glm::vec3 current_pos(current_state.x, current_state.y, current_state.z);
    glm::vec3 direction = glm::normalize(target - current_pos);
    float distance = glm::length(target - current_pos);
    
    // Scale velocity based on distance and aggressiveness
    float speed = glm::min(distance * pathfinding_aggressiveness_, MAX_VELOCITY);
    return direction * speed;
}

bool PathfindingAI::hasLandedOnTarget(const DroneState& current_state, const World& world) const {
    const Obstacle* target_building = world.getTargetBuilding();
    if (!target_building) {
        return false;
    }
    
    // Check if drone is on top of the target building
    float drone_x = current_state.x;
    float drone_y = current_state.y;
    float drone_z = current_state.z;
    
    float building_x = target_building->x;
    float building_y = target_building->y;
    float building_top_z = target_building->z + target_building->height;
    
    // Check horizontal distance (within building radius)
    float horizontal_distance = sqrt((drone_x - building_x) * (drone_x - building_x) + 
                                   (drone_y - building_y) * (drone_y - building_y));
    
    // Check vertical distance (close to building top)
    float vertical_distance = abs(drone_z - building_top_z);
    
    // Drone is considered landed if:
    // 1. Within building radius horizontally
    // 2. Close to building top vertically (within 5 units)
    // 3. Moving slowly (velocity < 5 units)
    float velocity = sqrt(current_state.vx * current_state.vx + 
                         current_state.vy * current_state.vy + 
                         current_state.vz * current_state.vz);
    
    return horizontal_distance <= target_building->radius && 
           vertical_distance <= 5.0f && 
           velocity < 5.0f;
}
