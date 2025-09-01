#pragma once

#include "../Drone.h"
#include "../World.h"
#include <glm/glm.hpp>
#include <vector>

struct ObstacleInfo {
    glm::vec3 position;
    float radius;
    float height;
    float distance;
    glm::vec3 direction;  // Direction from drone to obstacle
    bool is_threat;       // Whether this obstacle requires immediate action
};

struct AvoidanceVector {
    glm::vec3 direction;
    float strength;
    bool is_active;
};

class ObstacleAvoidance {
public:
    ObstacleAvoidance();
    
    // Main avoidance function
    AvoidanceVector calculateAvoidanceVector(const DroneState& drone_state, const World& world, const glm::vec3& desired_direction);
    
    // Obstacle detection
    std::vector<ObstacleInfo> detectNearbyObstacles(const DroneState& drone_state, const World& world, float max_distance);
    bool isCollisionImminent(const DroneState& drone_state, const World& world, float time_horizon);
    
    // Avoidance strategies
    glm::vec3 calculatePotentialField(const DroneState& drone_state, const std::vector<ObstacleInfo>& obstacles, const glm::vec3& goal);
    glm::vec3 calculateRepulsiveForce(const DroneState& drone_state, const ObstacleInfo& obstacle);
    glm::vec3 calculateAttractiveForce(const DroneState& drone_state, const glm::vec3& goal);
    
    // Configuration
    void setDetectionRadius(float radius) { detection_radius_ = radius; }
    void setSafetyDistance(float distance) { safety_distance_ = distance; }
    void setAvoidanceStrength(float strength) { avoidance_strength_ = strength; }
    void setTimeHorizon(float horizon) { time_horizon_ = horizon; }
    
    // Utility functions
    float calculateTimeToCollision(const DroneState& drone_state, const ObstacleInfo& obstacle) const;
    bool isObstacleInPath(const DroneState& drone_state, const glm::vec3& target, const ObstacleInfo& obstacle) const;
    
private:
    // Obstacle analysis
    void analyzeObstacles(std::vector<ObstacleInfo>& obstacles, const DroneState& drone_state, const glm::vec3& desired_direction);
    void prioritizeObstacles(std::vector<ObstacleInfo>& obstacles);
    bool isObstacleThreatening(const DroneState& drone_state, const ObstacleInfo& obstacle, const glm::vec3& desired_direction) const;
    
    // Avoidance calculations
    glm::vec3 calculateOptimalAvoidanceDirection(const std::vector<ObstacleInfo>& obstacles, const glm::vec3& desired_direction);
    glm::vec3 calculateLateralAvoidance(const DroneState& drone_state, const ObstacleInfo& obstacle);
    glm::vec3 calculateVerticalAvoidance(const DroneState& drone_state, const ObstacleInfo& obstacle);
    
    // Predictive avoidance
    glm::vec3 predictFuturePosition(const DroneState& drone_state, float time_ahead) const;
    bool willCollideInFuture(const DroneState& drone_state, const ObstacleInfo& obstacle, float time_ahead) const;
    
    // Configuration parameters
    float detection_radius_;
    float safety_distance_;
    float avoidance_strength_;
    float time_horizon_;
    float repulsive_force_range_;
    float attractive_force_range_;
    
    // Constants
    static constexpr float DEFAULT_DETECTION_RADIUS = 100.0f;
    static constexpr float DEFAULT_SAFETY_DISTANCE = 15.0f;
    static constexpr float DEFAULT_AVOIDANCE_STRENGTH = 1.0f;
    static constexpr float DEFAULT_TIME_HORIZON = 2.0f;
    static constexpr float DEFAULT_REPULSIVE_FORCE_RANGE = 50.0f;
    static constexpr float DEFAULT_ATTRACTIVE_FORCE_RANGE = 200.0f;
    static constexpr float MIN_OBSTACLE_DISTANCE = 5.0f;
    static constexpr float MAX_AVOIDANCE_STRENGTH = 5.0f;
};
