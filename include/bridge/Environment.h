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
    std::vector<cv::Point3f> path_trace;        // 3D path trace
    std::vector<cv::Point2f> path_trace_2d;     // 2D path trace (backward compatibility)
    float final_altitude;
    float altitude_error;                        // Difference between final and goal altitude
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
    float pitch_rate_scale = 1.0f;      // Scale factor for pitch rate actions
    float roll_rate_scale = 1.0f;       // Scale factor for roll rate actions
    float vertical_thrust_scale = 1.0f; // Scale factor for vertical thrust actions
    bool enable_safety_checks = true;    // Enable collision and constraint checks
    bool enable_action_logging = true;   // Enable action logging for debugging
    int action_log_frequency = 10;       // How often to log actions (steps)
    
    // 3D specific parameters
    bool enable_3d_mode = false;         // Whether to use 3D mode
    bool enable_3d_pathfinding = false;  // Whether to use 3D pathfinding
    float max_altitude = 200.0f;         // Maximum allowed altitude
    float min_altitude = 0.0f;           // Minimum allowed altitude
    float altitude_safety_margin = 10.0f; // Safety margin for altitude
    
    // Reward shaping
    float goal_reward;
    float collision_penalty;
    float progress_reward;
    float time_penalty;
    float safety_margin_penalty;
    float altitude_reward = 1.0f;        // Reward for maintaining good altitude
    float altitude_penalty = -2.0f;      // Penalty for altitude violations
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
    void set3DMode(bool enable) { config_.enable_3d_mode = enable; }
    void set3DPathfinding(bool enable) { config_.enable_3d_pathfinding = enable; }
    
    // Statistics
    const std::vector<EpisodeResult>& getEpisodeHistory() const { return episode_history_; }
    float getAverageEpisodeLength() const;
    float getSuccessRate() const;
    float getAverageAltitudeError() const;
    
    // Pathfinding integration
    std::vector<cv::Point3f> getOptimalPath3D() const;
    std::vector<cv::Point2f> getOptimalPath() const; // Backward compatibility
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
        
        // 3D action statistics
        int pitch_up_actions = 0;
        int pitch_down_actions = 0;
        int roll_left_actions = 0;
        int roll_right_actions = 0;
        int thrust_up_actions = 0;
        int thrust_down_actions = 0;
        int combined_3d_actions = 0;
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
        
        // 3D specific rewards
        float altitude_reward = 0.0f;
        float altitude_penalty = 0.0f;
        float vertical_progress_reward = 0.0f;
        float clearance_reward = 0.0f;
        
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
    std::vector<cv::Point3f> path_trace_3d_;        // 3D path trace
    std::vector<cv::Point2f> path_trace_2d_;        // 2D path trace (backward compatibility)
    cv::Point3f episode_start_;
    cv::Point3f episode_goal_;
    
    // History
    std::vector<EpisodeResult> episode_history_;
    
    // Pathfinding integration
    bool use_pathfinding_;
    std::string pathfinding_algorithm_;
    std::vector<cv::Point3f> optimal_path_3d_;      // 3D optimal path
    std::vector<cv::Point2f> optimal_path_2d_;      // 2D optimal path (backward compatibility)
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
    
    // 3D specific reward components
    float getAltitudeReward() const;
    float getVerticalProgressReward() const;
    float getClearanceReward() const;
    
    // Pathfinding helpers
    void computeOptimalPath();
    void computeOptimalPath3D();
    float getDistanceToPath() const;
    float getDistanceToPath3D() const;
    cv::Point3f getNextWaypoint3D() const;
    cv::Point2f getNextWaypoint() const; // Backward compatibility
    bool hasReachedWaypoint() const;
    void updateWaypointProgress();
    
    // 3D specific helpers
    bool is3DMode() const { return config_.enable_3d_mode; }
    bool is3DPathfindingEnabled() const { return config_.enable_3d_pathfinding; }
    void update3DActionStats(agent::Action action);
    float calculate3DDistance(const cv::Point3f& p1, const cv::Point3f& p2) const;
};

} // namespace bridge
