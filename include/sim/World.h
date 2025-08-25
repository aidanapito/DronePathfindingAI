#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <random>
#include <memory>
#include <queue>
#include <unordered_set>

namespace sim {

enum class MapType {
    MAZE,
    CORRIDOR,
    OPEN_FIELD,
    OBSTACLE_COURSE
};

struct Obstacle {
    cv::Point2f position;
    float radius;
    bool is_moving;
    cv::Point2f velocity;
    float max_speed;
};

// A* pathfinding node
struct PathNode {
    int x, y;
    float g_cost;  // Cost from start to current node
    float h_cost;  // Heuristic cost from current node to goal
    float f_cost;  // Total cost (g + h)
    std::pair<int, int> parent;  // Parent node coordinates
    
    PathNode(int x, int y, float g, float h, std::pair<int, int> parent = {-1, -1})
        : x(x), y(y), g_cost(g), h_cost(h), f_cost(g + h), parent(parent) {}
    
    bool operator>(const PathNode& other) const {
        return f_cost > other.f_cost;
    }
};

class World {
public:
    World(int width = 800, int height = 600);
    ~World() = default;

    // Map generation
    void generateMap(MapType type, int seed = -1);
    void addRandomObstacles(int count, float min_radius, float max_radius);
    void addMovingObstacles(int count, float min_radius, float max_radius, float max_speed);
    
    // World simulation
    void update(float dt);
    void render(cv::Mat& output);
    
    // Collision detection
    bool checkCollision(const cv::Point2f& position, float radius) const;
    bool isInBounds(const cv::Point2f& position) const;
    
    // Pathfinding
    std::vector<cv::Point2f> findPathAStar(const cv::Point2f& start, const cv::Point2f& goal, float grid_size = 10.0f);
    std::vector<cv::Point2f> findPathFloodFill(const cv::Point2f& start, const cv::Point2f& goal, float grid_size = 10.0f);
    bool isValidGridPosition(int x, int y, int grid_width, int grid_height) const;
    float calculateHeuristic(int x1, int y1, int x2, int y2) const;
    std::vector<std::pair<int, int>> getNeighbors(int x, int y, int grid_width, int grid_height) const;
    
    // Getters
    cv::Size getSize() const { return cv::Size(width_, height_); }
    const std::vector<Obstacle>& getObstacles() const { return obstacles_; }
    cv::Point2f getStartPosition() const { return start_position_; }
    cv::Point2f getGoalPosition() const { return goal_position_; }
    const std::vector<std::vector<bool>>& getOccupancyGrid() const { return occupancy_grid_; }
    
    // Setters
    void setStartPosition(const cv::Point2f& pos) { start_position_ = pos; }
    void setGoalPosition(const cv::Point2f& pos) { goal_position_ = pos; }
    
    // Domain randomization
    void randomizeLighting();
    void randomizeTextures();

private:
    void generateMaze();
    void generateCorridor();
    void generateOpenField();
    void generateObstacleCourse();
    
    void updateMovingObstacles(float dt);
    void bounceObstaclesOffWalls();
    void updateOccupancyGrid();
    
    int width_, height_;
    std::vector<Obstacle> obstacles_;
    std::vector<std::vector<bool>> occupancy_grid_;
    
    cv::Point2f start_position_;
    cv::Point2f goal_position_;
    
    std::mt19937 rng_;
    cv::Mat background_texture_;
    
    // Domain randomization parameters
    float lighting_brightness_;
    float lighting_contrast_;
    cv::Mat noise_texture_;
};

} // namespace sim
