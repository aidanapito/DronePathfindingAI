#pragma once

#include "../sim/World.h"
#include "../sim/Drone.h"
#include "../agent/Agent.h"
#include <memory>
#include <vector>

namespace bridge {

struct EpisodeResult {
    int steps;
    float total_reward;
    bool success;
    float final_distance;
    std::vector<cv::Point2f> path_trace;
};

struct EnvironmentConfig {
    int max_steps_per_episode;
    float time_step;
    bool render_episodes;
    bool record_video;
    std::string video_output_path;
    
    // Action execution parameters
    float throttle_scale = 1.0f;        // Scale factor for throttle actions
    float yaw_rate_scale = 1.0f;        // Scale factor for yaw rate actions
    bool enable_safety_checks = true;    // Enable collision and constraint checks
    bool enable_action_logging = true;   // Enable action logging for debugging
    int action_log_frequency = 10;       // How often to log actions (steps)
    
    // Reward shaping
    float goal_reward;
    float collision_penalty;
    float progress_reward;
    float time_penalty;
    float safety_margin_penalty;
};

class Environment {
public:
    Environment(const EnvironmentConfig& config);
    ~Environment() = default;

    // Episode management
    void reset();
    EpisodeResult runEpisode(std::shared_ptr<agent::Agent> agent);
    void step(std::shared_ptr<agent::Agent> agent);
    
    // Environment state
    bool isDone() const;
    
    // Episode information
    int getEpisodeCount() const { return episode_count_; }
    int getCurrentStep() const { return current_step_; }
    
    // Current state
    agent::Observation getCurrentObservation() const;
    float getCurrentReward() const;
    
    // Configuration
    void setAgent(std::shared_ptr<agent::Agent> agent);
    void setWorld(std::shared_ptr<sim::World> world);
    void setDrone(std::shared_ptr<sim::Drone> drone);
    
    // Statistics
    const std::vector<EpisodeResult>& getEpisodeHistory() const { return episode_history_; }
    float getAverageEpisodeLength() const;
    float getSuccessRate() const;
    
    // Pathfinding integration
    std::vector<cv::Point2f> getOptimalPath() const;
    void setUsePathfinding(bool use) { use_pathfinding_ = use; }
    bool getUsePathfinding() const { return use_pathfinding_; }
    void setPathfindingAlgorithm(const std::string& algorithm) { pathfinding_algorithm_ = algorithm; }
    std::string getPathfindingAlgorithm() const { return pathfinding_algorithm_; }
    int getCurrentWaypointIndex() const { return current_waypoint_index_; }
    
    // Action execution statistics
    struct ActionStats {
        int total_actions = 0;
        int forward_actions = 0;
        int left_turn_actions = 0;
        int right_turn_actions = 0;
        int idle_actions = 0;
        int blocked_actions = 0;
        int scaled_actions = 0;
        int emergency_stops = 0;
    };
    ActionStats getActionStats() const { return action_stats_; }
    void printActionStats() const;
    
    // Reward analysis
    struct RewardBreakdown {
        float goal_reward = 0.0f;
        float collision_penalty = 0.0f;
        float progress_reward = 0.0f;
        float time_penalty = 0.0f;
        float safety_penalty = 0.0f;
        float directional_reward = 0.0f;
        float path_following_reward = 0.0f;
        float efficiency_reward = 0.0f;
        float boundary_reward = 0.0f;
        float speed_reward = 0.0f;
        float proximity_reward = 0.0f;
        float action_reward = 0.0f;
        float total_reward = 0.0f;
    };
    RewardBreakdown getRewardBreakdown() const;

private:
    EnvironmentConfig config_;
    
    std::shared_ptr<sim::World> world_;
    std::shared_ptr<sim::Drone> drone_;
    std::shared_ptr<agent::Agent> agent_;
    
    // Episode state
    int current_step_;
    int episode_count_;
    float cumulative_reward_;
    std::vector<cv::Point2f> path_trace_;
    cv::Point2f episode_start_;
    cv::Point2f episode_goal_;
    
    // History
    std::vector<EpisodeResult> episode_history_;
    
    // Pathfinding integration
    bool use_pathfinding_;
    std::string pathfinding_algorithm_;
    std::vector<cv::Point2f> optimal_path_;
    int current_waypoint_index_;
    
    // Action execution statistics
    ActionStats action_stats_;
    
    // Last action tracking for reward shaping
    agent::Action last_action_;
    
    // Helper methods
    float calculateReward() const;
    bool checkEpisodeTermination() const;
    void updatePathTrace();
    agent::Observation createObservation() const;
    
    // Reward components
    float getGoalReward() const;
    float getCollisionPenalty() const;
    float getProgressReward() const;
    float getTimePenalty() const;
    float getSafetyMarginPenalty() const;
    
    // Pathfinding helpers
    void computeOptimalPath();
    float getDistanceToPath() const;
    cv::Point2f getNextWaypoint() const;
    bool hasReachedWaypoint() const;
    void updateWaypointProgress();
};

} // namespace bridge
