#pragma once

#include "Agent.h"
#include <unordered_map>
#include <vector>
#include <deque>
#include <set>

namespace agent {

struct QState {
    int grid_x, grid_y;         // Grid position
    int heading_bucket;         // Quantized heading (0-7 for 45° increments)
    
    bool operator==(const QState& other) const {
        return grid_x == other.grid_x && 
               grid_y == other.grid_y && 
               heading_bucket == other.heading_bucket;
    }
};

// Hash function for QState
struct QStateHash {
    std::size_t operator()(const QState& state) const {
        return std::hash<int>()(state.grid_x) ^ 
               (std::hash<int>()(state.grid_y) << 1) ^ 
               (std::hash<int>()(state.heading_bucket) << 2);
    }
};

// Path memory structure
struct PathNode {
    cv::Point2f position;
    float heading;
    int step_count;
    float reward;
    
    PathNode(const cv::Point2f& pos, float h, int step, float r)
        : position(pos), heading(h), step_count(step), reward(r) {}
};

class QLearningAgent : public Agent {
public:
    QLearningAgent(const AgentConfig& config);
    ~QLearningAgent() override = default;

    // Agent interface implementation
    Action selectAction(const Observation& obs, const sim::Drone& drone) override;
    void updatePolicy(const Observation& obs, Action action, 
                     float reward, const Observation& next_obs, bool done) override;
    void reset() override;
    
    // Q-learning specific
    void updateQValue(const QState& state, Action action, float reward, 
                     const QState& next_state, bool done);
    float getQValue(const QState& state, Action action) const;
    void setQValue(const QState& state, Action action, float value);
    
    // Action selection
    Action epsilonGreedyAction(const QState& state, const std::vector<Action>& valid_actions);
    Action greedyAction(const QState& state, const std::vector<Action>& valid_actions);
    Action selectExplorationAction(const QState& state, const std::vector<Action>& valid_actions);
    Action selectBacktrackAction(const cv::Point2f& current_pos, float current_heading, const std::vector<Action>& valid_actions);
    Action selectPanicAction(const QState& state, const std::vector<Action>& valid_actions);
    Action selectGoalSeekingAction(const QState& state, const std::vector<Action>& valid_actions);
    
    // Model persistence
    void saveModel(const std::string& path) override;
    void loadModel(const std::string& path) override;
    
    // Statistics
    size_t getQTableSize() const { return q_table_.size(); }
    float getAverageQValue() const;
    float getMaxQValue(const QState& state) const;
    
    // Debug information
    std::string getDebugInfo() const;
    
    // Pathfinding integration
    void setOptimalPath(const std::vector<cv::Point2f>& path);
    cv::Point2f getNextWaypointDirection(const cv::Point2f& current_pos) const;

private:
    std::unordered_map<QState, std::vector<float>, QStateHash> q_table_;
    
    // Q-learning parameters
    float alpha_;                // Learning rate
    float gamma_;                // Discount factor
    float epsilon_;              // Exploration rate
    
    // State discretization
    int grid_resolution_;        // Grid cell size
    int num_heading_buckets_;    // Number of heading buckets
    
    // Path memory and loop detection
    std::deque<PathNode> path_history_;          // Recent path history
    std::set<std::pair<int, int>> visited_cells_; // Visited grid cells
    int stuck_counter_;                          // Counter for stuck detection
    int last_progress_step_;                     // Last step with progress
    float last_best_distance_;                   // Best distance achieved
    bool is_exploring_;                          // Flag for exploration mode
    int exploration_steps_;                      // Steps in exploration mode
    std::vector<cv::Point2f> backtrack_path_;    // Path for backtracking
    bool is_backtracking_;                       // Flag for backtracking mode
    bool is_panic_mode_;                         // Flag for panic mode (extreme stuck)
    int panic_counter_;                          // Counter for panic mode
    bool is_goal_seeking_;                       // Flag for goal-seeking navigation mode
    int wall_following_steps_;                   // Counter for wall-following steps
    
    // Pathfinding integration
    std::vector<cv::Point2f> optimal_path_;
    int current_waypoint_index_;
    bool use_pathfinding_;
    
    // Configuration for stuck detection
    static constexpr int MAX_PATH_HISTORY = 100;     // Maximum path history size
    static constexpr int STUCK_THRESHOLD = 50;       // Steps without progress to consider stuck
    static constexpr int LOOP_DETECTION_WINDOW = 20; // Window for loop detection
    static constexpr int EXPLORATION_DURATION = 30;  // Steps to explore when stuck
    static constexpr float PROGRESS_THRESHOLD = 5.0f; // Distance improvement threshold
    static constexpr float BACKTRACK_DISTANCE = 50.0f; // Distance to backtrack
    static constexpr int PANIC_THRESHOLD = 60;       // Steps without progress to enter panic mode (reduced from 100)
    static constexpr int PANIC_DURATION = 50;         // Steps to stay in panic mode
    static constexpr float CIRCULAR_MOVEMENT_THRESHOLD = 0.5f; // Threshold for circular movement detection
    static constexpr int MIN_CIRCULAR_RADIUS = 20;   // Minimum radius for circular movement detection
    static constexpr int MAX_CIRCULAR_RADIUS = 100;  // Maximum radius for circular movement detection
    
    // Helper methods
    QState discretizeState(const Observation& obs) const;
    Action epsilonGreedyAction(const QState& state);
    Action greedyAction(const QState& state);
    void initializeQValue(const QState& state);
    
    // State conversion
    std::pair<int, int> worldToGrid(const cv::Point2f& world_pos) const;
    int headingToBucket(float heading) const;
    
    // Path memory and loop detection
    void updatePathHistory(const cv::Point2f& position, float heading, float reward);
    bool detectLoop() const;
    bool isStuck(float current_distance) const;
    Action selectExplorationAction(const QState& state);
    Action selectBacktrackAction(const cv::Point2f& current_pos, float current_heading);
    void updateBacktrackPath(const cv::Point2f& position, float current_heading);
    float calculateProgress(float current_distance);
    void resetStuckDetection();
    bool shouldTerminateBacktracking(float current_distance) const;

    // Stuck detection and recovery
    void updateStuckDetection(float current_distance);
    bool shouldTerminateExploration(float current_distance);
    bool shouldTerminateBacktracking(float current_distance);
    Action selectPanicAction(const Observation& obs, const std::vector<Action>& valid_actions);
    Action selectExplorationAction(const Observation& obs, const std::vector<Action>& valid_actions);
};

} // namespace agent
