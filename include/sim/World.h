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
    OBSTACLE_COURSE,
    SKYSCRAPER,      // New 3D map type
    UNDERWATER,      // New 3D map type
    MOUNTAIN_PASS    // New 3D map type
};

struct Obstacle {
    cv::Point3f position;      // x, y, z in world coordinates
    float radius;
    float height;               // Height of the obstacle
    bool is_moving;
    cv::Point3f velocity;      // 3D velocity
    float max_speed;
    bool is_vertical;           // Whether obstacle extends vertically
};

// 3D A* pathfinding node
struct PathNode3D {
    int x, y, z;
    float g_cost;  // Cost from start to current node
    float h_cost;  // Heuristic cost from current node to goal
    float f_cost;  // Total cost (g + h)
    std::tuple<int, int, int> parent;  // Parent node coordinates
    
    PathNode3D(int x, int y, int z, float g, float h, 
               std::tuple<int, int, int> parent = {-1, -1, -1})
        : x(x), y(y), z(z), g_cost(g), h_cost(h), f_cost(g + h), parent(parent) {}
    
    bool operator>(const PathNode3D& other) const {
        return f_cost > other.f_cost;
    }
};

class World {
public:
    World(int width = 800, int height = 600, int depth = 400);
    ~World() = default;

    // Map generation
    void generateMap(MapType type, int seed = -1);
    void addRandomObstacles(int count, float min_radius, float max_radius, 
                           float min_height = 10.0f, float max_height = 100.0f);
    void addMovingObstacles(int count, float min_radius, float max_radius, 
                           float max_speed, float min_height = 10.0f, float max_height = 100.0f);
    void addVerticalObstacles(int count, float min_radius, float max_radius, 
                             float min_height, float max_height);
    
    // World simulation
    void update(float dt);
    void render(cv::Mat& output);
    void render3D(cv::Mat& output, const cv::Point3f& camera_pos, 
                  const cv::Point3f& camera_target);
    
    // Collision detection
    bool checkCollision(const cv::Point3f& position, float radius) const;
    bool checkCollision(const cv::Point2f& position, float radius) const; // Backward compatibility
    bool isInBounds(const cv::Point3f& position) const;
    bool isInBounds(const cv::Point2f& position) const; // Backward compatibility
    
    // Pathfinding
    std::vector<cv::Point3f> findPathAStar3D(const cv::Point3f& start, const cv::Point3f& goal, 
                                             float grid_size = 10.0f);
    std::vector<cv::Point2f> findPathAStar(const cv::Point2f& start, const cv::Point2f& goal, 
                                           float grid_size = 10.0f); // Backward compatibility
    std::vector<cv::Point3f> findPathFloodFill3D(const cv::Point3f& start, const cv::Point3f& goal, 
                                                 float grid_size = 10.0f);
    std::vector<cv::Point2f> findPathFloodFill(const cv::Point2f& start, const cv::Point2f& goal, 
                                               float grid_size = 10.0f); // Backward compatibility
    
    // 3D pathfinding helpers
    bool isValidGridPosition3D(int x, int y, int z, int grid_width, int grid_height, int grid_depth) const;
    float calculateHeuristic3D(int x1, int y1, int z1, int x2, int y2, int z2) const;
    std::vector<std::tuple<int, int, int>> getNeighbors3D(int x, int y, int z, 
                                                          int grid_width, int grid_height, int grid_depth) const;
    
    // Legacy 2D helpers (backward compatibility)
    bool isValidGridPosition(int x, int y, int grid_width, int grid_height) const;
    float calculateHeuristic(int x1, int y1, int x2, int y2) const;
    std::vector<std::pair<int, int>> getNeighbors(int x, int y, int grid_width, int grid_height) const;
    
    // Getters
    cv::Size3i getSize() const { return cv::Size3i(width_, height_, depth_); }
    cv::Size getSize2D() const { return cv::Size(width_, height_); } // Backward compatibility
    const std::vector<Obstacle>& getObstacles() const { return obstacles_; }
    cv::Point3f getStartPosition() const { return start_position_; }
    cv::Point3f getGoalPosition() const { return goal_position_; }
    cv::Point2f getStartPosition2D() const { return cv::Point2f(start_position_.x, start_position_.y); } // Backward compatibility
    cv::Point2f getGoalPosition2D() const { return cv::Point2f(goal_position_.x, goal_position_.y); } // Backward compatibility
    const std::vector<std::vector<std::vector<bool>>>& get3DOccupancyGrid() const { return occupancy_grid_3d_; }
    const std::vector<std::vector<bool>>& getOccupancyGrid() const { return occupancy_grid_2d_; } // Backward compatibility
    
    // Setters
    void setStartPosition(const cv::Point3f& pos) { start_position_ = pos; }
    void setGoalPosition(const cv::Point3f& pos) { goal_position_ = pos; }
    void setStartPosition(const cv::Point2f& pos) { start_position_ = cv::Point3f(pos.x, pos.y, 0); } // Backward compatibility
    void setGoalPosition(const cv::Point2f& pos) { goal_position_ = cv::Point3f(pos.x, pos.y, 0); } // Backward compatibility
    
    // Domain randomization
    void randomizeLighting();
    void randomizeTextures();
    void randomize3DEnvironment();

private:
    void generateMaze();
    void generateCorridor();
    void generateOpenField();
    void generateObstacleCourse();
    void generateSkyscraper();      // New 3D map generation
    void generateUnderwater();      // New 3D map generation
    void generateMountainPass();    // New 3D map generation
    
    void updateMovingObstacles(float dt);
    void bounceObstaclesOffWalls();
    void updateOccupancyGrid();
    void update3DOccupancyGrid();
    
    int width_, height_, depth_;
    std::vector<Obstacle> obstacles_;
    std::vector<std::vector<bool>> occupancy_grid_2d_;           // Legacy 2D grid
    std::vector<std::vector<std::vector<bool>>> occupancy_grid_3d_; // New 3D grid
    
    cv::Point3f start_position_;
    cv::Point3f goal_position_;
    
    std::mt19937 rng_;
    cv::Mat background_texture_;
    
    // Domain randomization parameters
    float lighting_brightness_;
    float lighting_contrast_;
    cv::Mat noise_texture_;
    
    // 3D environment parameters
    float ground_level_;
    float max_building_height_;
    float terrain_variation_;
};

} // namespace sim
