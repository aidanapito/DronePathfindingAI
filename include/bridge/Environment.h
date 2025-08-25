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
    float getCurrentReward() const;
    agent::Observation getCurrentObservation() const;
    
    // Configuration
    void setAgent(std::shared_ptr<agent::Agent> agent);
    void setWorld(std::shared_ptr<sim::World> world);
    void setDrone(std::shared_ptr<sim::Drone> drone);
    
    // Statistics
    const std::vector<EpisodeResult>& getEpisodeHistory() const { return episode_history_; }
    float getAverageEpisodeLength() const;
    float getSuccessRate() const;

private:
    EnvironmentConfig config_;
    
    std::shared_ptr<sim::World> world_;
    std::shared_ptr<sim::Drone> drone_;
    std::shared_ptr<agent::Agent> agent_;
    
    // Episode state
    int current_step_;
    float cumulative_reward_;
    std::vector<cv::Point2f> path_trace_;
    cv::Point2f episode_start_;
    cv::Point2f episode_goal_;
    
    // History
    std::vector<EpisodeResult> episode_history_;
    
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
};

} // namespace bridge
