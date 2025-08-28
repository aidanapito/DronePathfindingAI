#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

namespace sim {
    class World;
    class Drone;
}

namespace agent {

enum class Action {
    // 2D actions (backward compatibility)
    THROTTLE_FORWARD,
    YAW_LEFT,
    YAW_RIGHT,
    IDLE,
    
    // New 3D actions
    PITCH_UP,           // Pitch up (nose up)
    PITCH_DOWN,         // Pitch down (nose down)
    ROLL_LEFT,          // Roll left (bank left)
    ROLL_RIGHT,         // Roll right (bank right)
    THRUST_UP,          // Increase altitude
    THRUST_DOWN,        // Decrease altitude
    
    // Combined 3D actions for more complex maneuvers
    FORWARD_AND_UP,     // Move forward while gaining altitude
    FORWARD_AND_DOWN,   // Move forward while losing altitude
    TURN_AND_CLIMB,     // Turn while climbing
    TURN_AND_DIVE       // Turn while diving
};

struct Observation {
    cv::Mat image;              // For Track B: 84x84 grayscale
    std::vector<float> grid;    // For Track A: 21x21 occupancy grid
    std::vector<float> grid_3d; // For 3D: 21x21x10 occupancy grid (10 height layers)
    
    // 2D orientation (backward compatibility)
    float heading;              // Current yaw angle
    float goal_direction;       // Direction to goal in 2D
    
    // 3D orientation
    float pitch;                // Current pitch angle
    float roll;                 // Current roll angle
    float pitch_rate;           // Current pitch rate
    float roll_rate;            // Current roll rate
    
    // Position and movement
    cv::Point3f position;       // Current 3D position
    cv::Point2f position_2d;    // Current 2D position (backward compatibility)
    float distance_to_goal;     // 3D distance to goal
    float altitude;             // Current altitude
    float vertical_velocity;    // Current vertical velocity
    
    // Goal information
    cv::Point3f goal_position;  // 3D goal position
    cv::Point2f goal_position_2d; // 2D goal position (backward compatibility)
    float goal_altitude;        // Goal altitude
    float altitude_to_goal;     // Vertical distance to goal
    
    // 3D specific
    bool is_in_altitude_bounds; // Whether drone is within altitude constraints
    float clearance_above;      // Distance to obstacle above
    float clearance_below;      // Distance to obstacle below
    float clearance_forward;    // Distance to obstacle forward
};

struct AgentConfig {
    bool use_vision;            // Track A vs Track B
    bool use_3d;                // Whether to use 3D mode
    int observation_stack;      // Number of frames to stack (Track B)
    float learning_rate;
    float discount_factor;
    float epsilon;              // For exploration
    int replay_buffer_size;
    
    // 3D specific parameters
    float max_altitude;         // Maximum allowed altitude
    float min_altitude;         // Minimum allowed altitude
    float altitude_safety_margin; // Safety margin for altitude
    bool enable_3d_pathfinding; // Whether to use 3D pathfinding
};

class Agent {
public:
    Agent(const AgentConfig& config);
    virtual ~Agent() = default;

    // Core interface
    virtual Action selectAction(const Observation& obs, const sim::Drone& drone) = 0;
    virtual void updatePolicy(const Observation& obs, Action action, 
                            float reward, const Observation& next_obs, bool done) = 0;
    virtual void reset() = 0;
    
    // Training interface
    virtual void saveModel(const std::string& path) = 0;
    virtual void loadModel(const std::string& path) = 0;
    
    // Configuration
    const AgentConfig& getConfig() const { return config_; }
    void setEpsilon(float epsilon) { config_.epsilon = epsilon; }
    void setUse3D(bool use_3d) { config_.use_3d = use_3d; }

protected:
    AgentConfig config_;
    
    // Helper methods
    std::vector<Action> getValidActions(const sim::Drone& drone) const;
    std::vector<Action> getValid3DActions(const sim::Drone& drone) const;
    bool isValidAction(Action action, const sim::Drone& drone) const;
    bool isValid3DAction(Action action, const sim::Drone& drone) const;
    
    // 3D action helpers
    bool shouldUse3DAction(const Observation& obs) const;
    Action convert2DTo3DAction(Action action_2d, const Observation& obs) const;
    Action selectOptimal3DAction(const Observation& obs, const sim::Drone& drone) const;
};

} // namespace agent
