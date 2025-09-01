#pragma once

#include "../Drone.h"
#include "../World.h"
#include "PathPlanner.h"
#include "ObstacleAvoidance.h"
#include <glm/glm.hpp>
#include <memory>

enum class AIMode {
    MANUAL,         // Human control
    FOLLOW_PATH,    // Follow a planned path
    EXPLORE,        // Explore the environment
    RETURN_HOME,    // Return to starting position
    AVOID_OBSTACLES // Just avoid obstacles while maintaining current direction
};

enum class AIState {
    IDLE,
    PLANNING_PATH,
    FOLLOWING_PATH,
    AVOIDING_OBSTACLE,
    COMPLETED_MISSION,
    ERROR
};

class PathfindingAI {
public:
    PathfindingAI();
    
    // Main AI update function
    DroneInput update(float delta_time, const DroneState& current_state, const World& world);
    
    // AI control
    void setMode(AIMode mode);
    AIMode getMode() const { return current_mode_; }
    AIState getState() const { return current_state_; }
    
    // Path planning
    void setTarget(float x, float y, float z);
    void setHomePosition(float x, float y, float z);
    void clearPath();
    
    // Configuration
    void setSafetyMargin(float margin) { safety_margin_ = margin; }
    void setPathfindingAggressiveness(float aggressiveness) { pathfinding_aggressiveness_ = aggressiveness; }
    
    // Status queries
    bool isPathValid() const;
    bool isTargetReached() const;
    float getDistanceToTarget() const;
    bool hasLandedOnTarget(const DroneState& current_state, const World& world) const;
    
    // Path planning
    void updatePath(const DroneState& current_state, const World& world);
    
    // Debug information
    const std::vector<glm::vec3>& getCurrentPath() const;
    glm::vec3 getCurrentTarget() const;
    size_t getCurrentWaypointIndex() const { return current_waypoint_index_; }

private:
    // AI components
    std::unique_ptr<PathPlanner> path_planner_;
    std::unique_ptr<ObstacleAvoidance> obstacle_avoidance_;
    
    // AI state
    AIMode current_mode_;
    AIState current_state_;
    
    // Target and path information
    glm::vec3 target_position_;
    glm::vec3 home_position_;
    std::vector<glm::vec3> current_path_;
    size_t current_waypoint_index_;
    
    // Configuration parameters
    float safety_margin_;
    float pathfinding_aggressiveness_;
    float target_reached_threshold_;
    float waypoint_reached_threshold_;
    
    // Internal methods
    DroneInput generateManualInput();
    DroneInput generatePathFollowingInput(const DroneState& current_state, const World& world, float delta_time);
    DroneInput generateObstacleAvoidanceInput(const DroneState& current_state, const World& world, float delta_time);
    DroneInput generateExplorationInput(const DroneState& current_state, const World& world, float delta_time);
    DroneInput generateReturnHomeInput(const DroneState& current_state, float delta_time);
    
    bool isWaypointReached(const DroneState& current_state, const glm::vec3& waypoint) const;
    void advanceToNextWaypoint();
    glm::vec3 calculateDesiredVelocity(const DroneState& current_state, const glm::vec3& target) const;
    
    // Constants
    static constexpr float DEFAULT_SAFETY_MARGIN = 10.0f;
    static constexpr float DEFAULT_PATHFINDING_AGGRESSIVENESS = 0.5f;
    static constexpr float DEFAULT_TARGET_REACHED_THRESHOLD = 5.0f;
    static constexpr float DEFAULT_WAYPOINT_REACHED_THRESHOLD = 10.0f;
    static constexpr float MAX_VELOCITY = 50.0f;
    static constexpr float TURNING_RATE = 2.0f;
};
