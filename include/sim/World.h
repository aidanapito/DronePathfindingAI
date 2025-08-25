#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <random>
#include <memory>

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
    
    // Getters
    cv::Size getSize() const { return cv::Size(width_, height_); }
    const std::vector<Obstacle>& getObstacles() const { return obstacles_; }
    cv::Point2f getStartPosition() const { return start_position_; }
    cv::Point2f getGoalPosition() const { return goal_position_; }
    
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
