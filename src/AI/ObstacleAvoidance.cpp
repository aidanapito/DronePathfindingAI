#include "AI/ObstacleAvoidance.h"
#include <iostream>
#include <algorithm>
#include <cmath>

ObstacleAvoidance::ObstacleAvoidance()
    : detection_radius_(DEFAULT_DETECTION_RADIUS)
    , safety_distance_(DEFAULT_SAFETY_DISTANCE)
    , avoidance_strength_(DEFAULT_AVOIDANCE_STRENGTH)
    , time_horizon_(DEFAULT_TIME_HORIZON)
    , repulsive_force_range_(DEFAULT_REPULSIVE_FORCE_RANGE)
    , attractive_force_range_(DEFAULT_ATTRACTIVE_FORCE_RANGE) {
}

AvoidanceVector ObstacleAvoidance::calculateAvoidanceVector(const DroneState& drone_state, const World& world, const glm::vec3& desired_direction) {
    // Detect nearby obstacles
    auto obstacles = detectNearbyObstacles(drone_state, world, detection_radius_);
    
    if (obstacles.empty()) {
        return AvoidanceVector{desired_direction, 0.0f, false};
    }
    
    // Analyze and prioritize obstacles
    analyzeObstacles(obstacles, drone_state, desired_direction);
    prioritizeObstacles(obstacles);
    
    // Check for imminent collisions
    if (isCollisionImminent(drone_state, world, time_horizon_)) {
        // Emergency avoidance
        glm::vec3 emergency_direction = calculateOptimalAvoidanceDirection(obstacles, desired_direction);
        return AvoidanceVector{emergency_direction, MAX_AVOIDANCE_STRENGTH, true};
    }
    
    // Calculate potential field
    glm::vec3 avoidance_direction = calculatePotentialField(drone_state, obstacles, desired_direction * attractive_force_range_);
    
    // Determine avoidance strength based on closest obstacle
    float closest_distance = obstacles.front().distance;
    float strength = 0.0f;
    
    if (closest_distance < safety_distance_) {
        strength = avoidance_strength_ * (1.0f - closest_distance / safety_distance_);
        strength = glm::clamp(strength, 0.0f, MAX_AVOIDANCE_STRENGTH);
    }
    
    return AvoidanceVector{avoidance_direction, strength, strength > 0.1f};
}

std::vector<ObstacleInfo> ObstacleAvoidance::detectNearbyObstacles(const DroneState& drone_state, const World& world, float max_distance) {
    std::vector<ObstacleInfo> obstacles;
    glm::vec3 drone_pos(drone_state.x, drone_state.y, drone_state.z);
    
    for (const auto& obstacle : world.getObstacles()) {
        glm::vec3 obstacle_pos(obstacle.x, obstacle.y, obstacle.z);
        glm::vec3 to_obstacle = obstacle_pos - drone_pos;
        float distance = glm::length(to_obstacle);
        
        if (distance <= max_distance) {
            ObstacleInfo info;
            info.position = obstacle_pos;
            info.radius = obstacle.radius;
            info.height = obstacle.height;
            info.distance = distance;
            info.direction = glm::normalize(to_obstacle);
            info.is_threat = false; // Will be set during analysis
            
            obstacles.push_back(info);
        }
    }
    
    return obstacles;
}

bool ObstacleAvoidance::isCollisionImminent(const DroneState& drone_state, const World& world, float time_horizon) {
    glm::vec3 future_pos = predictFuturePosition(drone_state, time_horizon);
    
    // Check if future position collides with any obstacle
    for (const auto& obstacle : world.getObstacles()) {
        glm::vec3 obstacle_pos(obstacle.x, obstacle.y, obstacle.z);
        float distance = glm::length(future_pos - obstacle_pos);
        
        if (distance < obstacle.radius + safety_distance_) {
            return true;
        }
    }
    
    return false;
}

glm::vec3 ObstacleAvoidance::calculatePotentialField(const DroneState& drone_state, const std::vector<ObstacleInfo>& obstacles, const glm::vec3& goal) {
    glm::vec3 drone_pos(drone_state.x, drone_state.y, drone_state.z);
    glm::vec3 total_force(0.0f);
    
    // Calculate attractive force to goal
    glm::vec3 attractive_force = calculateAttractiveForce(drone_state, goal);
    total_force += attractive_force;
    
    // Calculate repulsive forces from obstacles
    for (const auto& obstacle : obstacles) {
        glm::vec3 repulsive_force = calculateRepulsiveForce(drone_state, obstacle);
        total_force += repulsive_force;
    }
    
    // Normalize the total force
    float force_magnitude = glm::length(total_force);
    if (force_magnitude > 0.0f) {
        return glm::normalize(total_force);
    }
    
    return glm::vec3(0.0f, 1.0f, 0.0f); // Default forward direction
}

glm::vec3 ObstacleAvoidance::calculateRepulsiveForce(const DroneState& drone_state, const ObstacleInfo& obstacle) {
    glm::vec3 drone_pos(drone_state.x, drone_state.y, drone_state.z);
    glm::vec3 to_obstacle = obstacle.position - drone_pos;
    float distance = glm::length(to_obstacle);
    
    if (distance > repulsive_force_range_) {
        return glm::vec3(0.0f);
    }
    
    // Calculate repulsive force strength
    float force_strength = 0.0f;
    if (distance < safety_distance_) {
        force_strength = avoidance_strength_ * (1.0f - distance / safety_distance_);
    } else {
        force_strength = avoidance_strength_ * (1.0f - distance / repulsive_force_range_);
    }
    
    // Repulsive force points away from obstacle
    glm::vec3 repulsive_direction = -glm::normalize(to_obstacle);
    
    return repulsive_direction * force_strength;
}

glm::vec3 ObstacleAvoidance::calculateAttractiveForce(const DroneState& drone_state, const glm::vec3& goal) {
    glm::vec3 drone_pos(drone_state.x, drone_state.y, drone_state.z);
    glm::vec3 to_goal = goal - drone_pos;
    float distance = glm::length(to_goal);
    
    if (distance > attractive_force_range_) {
        distance = attractive_force_range_;
    }
    
    // Attractive force strength decreases with distance
    float force_strength = 1.0f - (distance / attractive_force_range_);
    glm::vec3 attractive_direction = glm::normalize(to_goal);
    
    return attractive_direction * force_strength;
}

float ObstacleAvoidance::calculateTimeToCollision(const DroneState& drone_state, const ObstacleInfo& obstacle) const {
    glm::vec3 drone_pos(drone_state.x, drone_state.y, drone_state.z);
    glm::vec3 drone_velocity(drone_state.vx, drone_state.vy, drone_state.vz);
    
    glm::vec3 to_obstacle = obstacle.position - drone_pos;
    float distance = glm::length(to_obstacle);
    
    // Calculate relative velocity towards obstacle
    float relative_velocity = glm::dot(drone_velocity, glm::normalize(to_obstacle));
    
    if (relative_velocity <= 0.0f) {
        return std::numeric_limits<float>::infinity(); // Moving away
    }
    
    float time_to_collision = (distance - obstacle.radius - safety_distance_) / relative_velocity;
    return time_to_collision;
}

bool ObstacleAvoidance::isObstacleInPath(const DroneState& drone_state, const glm::vec3& target, const ObstacleInfo& obstacle) const {
    glm::vec3 drone_pos(drone_state.x, drone_state.y, drone_state.z);
    glm::vec3 path_direction = glm::normalize(target - drone_pos);
    
    // Calculate distance from obstacle to path line
    glm::vec3 to_obstacle = obstacle.position - drone_pos;
    float projection = glm::dot(to_obstacle, path_direction);
    
    if (projection < 0.0f) {
        return false; // Obstacle is behind drone
    }
    
    glm::vec3 closest_point = drone_pos + path_direction * projection;
    float distance_to_path = glm::length(obstacle.position - closest_point);
    
    return distance_to_path < obstacle.radius + safety_distance_;
}

void ObstacleAvoidance::analyzeObstacles(std::vector<ObstacleInfo>& obstacles, const DroneState& drone_state, const glm::vec3& desired_direction) {
    for (auto& obstacle : obstacles) {
        // Check if obstacle is threatening
        obstacle.is_threat = isObstacleThreatening(drone_state, obstacle, desired_direction);
        
        // Calculate time to collision
        float ttc = calculateTimeToCollision(drone_state, obstacle);
        if (ttc < time_horizon_ && ttc > 0.0f) {
            obstacle.is_threat = true;
        }
    }
}

void ObstacleAvoidance::prioritizeObstacles(std::vector<ObstacleInfo>& obstacles) {
    // Sort by threat level and distance
    std::sort(obstacles.begin(), obstacles.end(), 
        [](const ObstacleInfo& a, const ObstacleInfo& b) {
            if (a.is_threat != b.is_threat) {
                return a.is_threat > b.is_threat; // Threats first
            }
            return a.distance < b.distance; // Closer obstacles first
        });
}

bool ObstacleAvoidance::isObstacleThreatening(const DroneState& drone_state, const ObstacleInfo& obstacle, const glm::vec3& desired_direction) const {
    // Check if obstacle is in the desired path
    glm::vec3 target(drone_state.x + desired_direction.x * 100.0f, 
                     drone_state.y + desired_direction.y * 100.0f, 
                     drone_state.z + desired_direction.z * 100.0f);
    if (isObstacleInPath(drone_state, target, obstacle)) {
        return true;
    }
    
    // Check if obstacle is very close
    if (obstacle.distance < safety_distance_) {
        return true;
    }
    
    // Check time to collision
    float ttc = calculateTimeToCollision(drone_state, obstacle);
    if (ttc < time_horizon_ && ttc > 0.0f) {
        return true;
    }
    
    return false;
}

glm::vec3 ObstacleAvoidance::calculateOptimalAvoidanceDirection(const std::vector<ObstacleInfo>& obstacles, const glm::vec3& desired_direction) {
    if (obstacles.empty()) {
        return desired_direction;
    }
    
    // Find the most threatening obstacle
    const ObstacleInfo& threat = obstacles.front();
    
    // Calculate avoidance direction perpendicular to obstacle direction
    glm::vec3 avoidance_direction = glm::cross(threat.direction, glm::vec3(0.0f, 0.0f, 1.0f));
    
    if (glm::length(avoidance_direction) < 0.1f) {
        // If perpendicular direction is too small, use vertical avoidance
        avoidance_direction = glm::vec3(0.0f, 0.0f, 1.0f);
    } else {
        avoidance_direction = glm::normalize(avoidance_direction);
    }
    
    // Blend with desired direction
    float blend_factor = 0.7f; // 70% avoidance, 30% desired direction
    return glm::normalize(avoidance_direction * blend_factor + desired_direction * (1.0f - blend_factor));
}

glm::vec3 ObstacleAvoidance::calculateLateralAvoidance(const DroneState& drone_state, const ObstacleInfo& obstacle) {
    // Calculate lateral avoidance (left/right)
    glm::vec3 drone_pos(drone_state.x, drone_state.y, drone_state.z);
    glm::vec3 to_obstacle = obstacle.position - drone_pos;
    
    // Project onto horizontal plane
    to_obstacle.z = 0.0f;
    to_obstacle = glm::normalize(to_obstacle);
    
    // Perpendicular direction
    glm::vec3 lateral_direction = glm::cross(to_obstacle, glm::vec3(0.0f, 0.0f, 1.0f));
    
    return glm::normalize(lateral_direction);
}

glm::vec3 ObstacleAvoidance::calculateVerticalAvoidance(const DroneState& drone_state, const ObstacleInfo& obstacle) {
    // Calculate vertical avoidance (up/down)
    glm::vec3 drone_pos(drone_state.x, drone_state.y, drone_state.z);
    float height_difference = obstacle.position.z - drone_pos.z;
    
    if (height_difference > 0.0f) {
        // Obstacle is above, go down
        return glm::vec3(0.0f, 0.0f, -1.0f);
    } else {
        // Obstacle is below, go up
        return glm::vec3(0.0f, 0.0f, 1.0f);
    }
}

glm::vec3 ObstacleAvoidance::predictFuturePosition(const DroneState& drone_state, float time_ahead) const {
    return glm::vec3(
        drone_state.x + drone_state.vx * time_ahead,
        drone_state.y + drone_state.vy * time_ahead,
        drone_state.z + drone_state.vz * time_ahead
    );
}

bool ObstacleAvoidance::willCollideInFuture(const DroneState& drone_state, const ObstacleInfo& obstacle, float time_ahead) const {
    glm::vec3 future_pos = predictFuturePosition(drone_state, time_ahead);
    float future_distance = glm::length(future_pos - obstacle.position);
    
    return future_distance < obstacle.radius + safety_distance_;
}
