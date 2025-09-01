#pragma once

#include "../World.h"
#include <glm/glm.hpp>
#include <vector>
#include <queue>
#include <unordered_set>
#include <memory>

struct PathNode {
    float x, y, z;
    float g_cost;  // Cost from start to this node
    float h_cost;  // Heuristic cost to goal
    float f_cost;  // Total cost (g + h)
    std::shared_ptr<PathNode> parent;
    
    PathNode(float x, float y, float z) : x(x), y(y), z(z), g_cost(0), h_cost(0), f_cost(0), parent(nullptr) {}
    
    bool operator==(const PathNode& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct PathNodeHash {
    std::size_t operator()(const std::shared_ptr<PathNode>& node) const {
        return std::hash<float>()(node->x) ^ std::hash<float>()(node->y) ^ std::hash<float>()(node->z);
    }
};

struct PathNodeEqual {
    bool operator()(const std::shared_ptr<PathNode>& a, const std::shared_ptr<PathNode>& b) const {
        return *a == *b;
    }
};

struct PathNodeCompare {
    bool operator()(const std::shared_ptr<PathNode>& a, const std::shared_ptr<PathNode>& b) const {
        return a->f_cost > b->f_cost;
    }
};

class PathPlanner {
public:
    PathPlanner();
    
    // Main pathfinding function
    std::vector<glm::vec3> findPath(const glm::vec3& start, const glm::vec3& goal, const World& world);
    
    // Path optimization
    std::vector<glm::vec3> optimizePath(const std::vector<glm::vec3>& path, const World& world);
    std::vector<glm::vec3> smoothPath(const std::vector<glm::vec3>& path, const World& world);
    
    // Configuration
    void setGridResolution(float resolution) { grid_resolution_ = resolution; }
    void setMaxSearchIterations(int iterations) { max_search_iterations_ = iterations; }
    void setSafetyMargin(float margin) { safety_margin_ = margin; }
    
    // Utility functions
    bool isPathValid(const std::vector<glm::vec3>& path, const World& world) const;
    float calculatePathLength(const std::vector<glm::vec3>& path) const;
    
private:
    // A* pathfinding implementation
    std::vector<glm::vec3> astarSearch(const glm::vec3& start, const glm::vec3& goal, const World& world);
    std::vector<std::shared_ptr<PathNode>> getNeighbors(const std::shared_ptr<PathNode>& node, const World& world);
    float calculateHeuristic(const glm::vec3& from, const glm::vec3& to) const;
    float calculateCost(const glm::vec3& from, const glm::vec3& to, const World& world) const;
    std::vector<glm::vec3> reconstructPath(const std::shared_ptr<PathNode>& goal_node) const;
    
    // Grid-based operations
    glm::vec3 snapToGrid(const glm::vec3& position) const;
    bool isPositionValid(const glm::vec3& position, const World& world) const;
    bool isLineOfSight(const glm::vec3& from, const glm::vec3& to, const World& world) const;
    
    // Path optimization helpers
    std::vector<glm::vec3> simplifyPath(const std::vector<glm::vec3>& path, const World& world);
    std::vector<glm::vec3> addIntermediatePoints(const std::vector<glm::vec3>& path, const World& world);
    
    // Configuration parameters
    float grid_resolution_;
    int max_search_iterations_;
    float safety_margin_;
    float line_of_sight_step_size_;
    
    // Constants
    static constexpr float DEFAULT_GRID_RESOLUTION = 10.0f;
    static constexpr int DEFAULT_MAX_SEARCH_ITERATIONS = 10000;
    static constexpr float DEFAULT_SAFETY_MARGIN = 5.0f;
    static constexpr float DEFAULT_LINE_OF_SIGHT_STEP_SIZE = 2.0f;
    static constexpr float MAX_PATH_SEGMENT_LENGTH = 50.0f;
};
