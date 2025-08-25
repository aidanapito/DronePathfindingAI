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
    THROTTLE_FORWARD,
    YAW_LEFT,
    YAW_RIGHT,
    IDLE
};

struct Observation {
    cv::Mat image;              // For Track B: 84x84 grayscale
    std::vector<float> grid;    // For Track A: 21x21 occupancy grid
    float heading;              // Current heading
    float goal_direction;       // Direction to goal
    float distance_to_goal;     // Distance to goal
    cv::Point2f position;      // Current drone position in world coordinates
};

struct AgentConfig {
    bool use_vision;            // Track A vs Track B
    int observation_stack;      // Number of frames to stack (Track B)
    float learning_rate;
    float discount_factor;
    float epsilon;              // For exploration
    int replay_buffer_size;
};

class Agent {
public:
    Agent(const AgentConfig& config);
    virtual ~Agent() = default;

    // Core interface
    virtual Action selectAction(const Observation& obs) = 0;
    virtual void updatePolicy(const Observation& obs, Action action, 
                            float reward, const Observation& next_obs, bool done) = 0;
    virtual void reset() = 0;
    
    // Training interface
    virtual void saveModel(const std::string& path) = 0;
    virtual void loadModel(const std::string& path) = 0;
    
    // Configuration
    const AgentConfig& getConfig() const { return config_; }
    void setEpsilon(float epsilon) { config_.epsilon = epsilon; }

protected:
    AgentConfig config_;
    
    // Helper methods
    std::vector<Action> getValidActions(const sim::Drone& drone) const;
    bool isValidAction(Action action, const sim::Drone& drone) const;
};

} // namespace agent
