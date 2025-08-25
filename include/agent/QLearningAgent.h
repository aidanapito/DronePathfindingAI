#pragma once

#include "Agent.h"
#include <unordered_map>
#include <vector>

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

class QLearningAgent : public Agent {
public:
    QLearningAgent(const AgentConfig& config);
    ~QLearningAgent() override = default;

    // Agent interface implementation
    Action selectAction(const Observation& obs) override;
    void updatePolicy(const Observation& obs, Action action, 
                     float reward, const Observation& next_obs, bool done) override;
    void reset() override;
    
    // Q-learning specific
    void updateQValue(const QState& state, Action action, float reward, 
                     const QState& next_state, bool done);
    float getQValue(const QState& state, Action action) const;
    void setQValue(const QState& state, Action action, float value);
    
    // Model persistence
    void saveModel(const std::string& path) override;
    void loadModel(const std::string& path) override;
    
    // Statistics
    size_t getQTableSize() const { return q_table_.size(); }
    float getAverageQValue() const;
    float getMaxQValue(const QState& state) const;

private:
    std::unordered_map<QState, std::vector<float>, QStateHash> q_table_;
    
    // Q-learning parameters
    float alpha_;                // Learning rate
    float gamma_;                // Discount factor
    float epsilon_;              // Exploration rate
    
    // State discretization
    int grid_resolution_;        // Grid cell size
    int num_heading_buckets_;    // Number of heading buckets
    
    // Helper methods
    QState discretizeState(const Observation& obs) const;
    Action epsilonGreedyAction(const QState& state);
    Action greedyAction(const QState& state);
    void initializeQValue(const QState& state);
    
    // State conversion
    std::pair<int, int> worldToGrid(const cv::Point2f& world_pos) const;
    int headingToBucket(float heading) const;
};

} // namespace agent
