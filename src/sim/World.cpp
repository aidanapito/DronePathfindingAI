#include "sim/World.h"
#include <iostream>

namespace sim {

World::World(int width, int height) 
    : width_(width), height_(height), 
      lighting_brightness_(1.0f), lighting_contrast_(1.0f) {
    // Initialize random number generator
    rng_.seed(std::random_device{}());
    
    // Initialize start and goal positions
    start_position_ = cv::Point2f(50.0f, height_ / 2.0f);
    goal_position_ = cv::Point2f(width_ - 50.0f, height_ / 2.0f);
    
    // Initialize occupancy grid
    occupancy_grid_.resize(height_, std::vector<bool>(width_, false));
}

void World::generateMap(MapType type, int seed) {
    if (seed >= 0) {
        rng_.seed(seed);
    }
    
    // Clear existing obstacles
    obstacles_.clear();
    
    switch (type) {
        case MapType::MAZE:
            generateMaze();
            break;
        case MapType::CORRIDOR:
            generateCorridor();
            break;
        case MapType::OPEN_FIELD:
            generateOpenField();
            break;
        case MapType::OBSTACLE_COURSE:
            generateObstacleCourse();
            break;
    }
}

void World::addRandomObstacles(int count, float min_radius, float max_radius) {
    std::uniform_real_distribution<float> radius_dist(min_radius, max_radius);
    std::uniform_real_distribution<float> x_dist(100.0f, width_ - 100.0f);
    std::uniform_real_distribution<float> y_dist(100.0f, height_ - 100.0f);
    
    for (int i = 0; i < count; ++i) {
        Obstacle obs;
        obs.position = cv::Point2f(x_dist(rng_), y_dist(rng_));
        obs.radius = radius_dist(rng_);
        obs.is_moving = false;
        obs.velocity = cv::Point2f(0.0f, 0.0f);
        obs.max_speed = 0.0f;
        
        obstacles_.push_back(obs);
    }
}

void World::addMovingObstacles(int count, float min_radius, float max_radius, float max_speed) {
    std::uniform_real_distribution<float> radius_dist(min_radius, max_radius);
    std::uniform_real_distribution<float> x_dist(100.0f, width_ - 100.0f);
    std::uniform_real_distribution<float> y_dist(100.0f, height_ - 100.0f);
    std::uniform_real_distribution<float> angle_dist(0.0f, 2.0f * M_PI);
    
    for (int i = 0; i < count; ++i) {
        Obstacle obs;
        obs.position = cv::Point2f(x_dist(rng_), y_dist(rng_));
        obs.radius = radius_dist(rng_);
        obs.is_moving = true;
        
        float angle = angle_dist(rng_);
        float speed = max_speed * (0.5f + 0.5f * (rng_() % 100) / 100.0f);
        obs.velocity = cv::Point2f(speed * cos(angle), speed * sin(angle));
        obs.max_speed = max_speed;
        
        obstacles_.push_back(obs);
    }
}

void World::update(float dt) {
    updateMovingObstacles(dt);
    bounceObstaclesOffWalls();
}

void World::render(cv::Mat& output) {
    // Create a white background
    output = cv::Mat(height_, width_, CV_8UC3, cv::Scalar(255, 255, 255));
    
    // Draw obstacles
    for (const auto& obs : obstacles_) {
        cv::circle(output, obs.position, obs.radius, cv::Scalar(0, 0, 0), -1);
    }
    
    // Draw start and goal
    cv::circle(output, start_position_, 10, cv::Scalar(0, 255, 0), -1);
    cv::circle(output, goal_position_, 10, cv::Scalar(0, 0, 255), -1);
}

bool World::checkCollision(const cv::Point2f& position, float radius) const {
    for (const auto& obs : obstacles_) {
        float distance = cv::norm(position - obs.position);
        if (distance < (radius + obs.radius)) {
            return true;
        }
    }
    return false;
}

bool World::isInBounds(const cv::Point2f& position) const {
    return position.x >= 0 && position.x < width_ && 
           position.y >= 0 && position.y < height_;
}

// These methods are already defined inline in the header

void World::randomizeLighting() {
    std::uniform_real_distribution<float> brightness_dist(0.7f, 1.3f);
    std::uniform_real_distribution<float> contrast_dist(0.8f, 1.2f);
    
    lighting_brightness_ = brightness_dist(rng_);
    lighting_contrast_ = contrast_dist(rng_);
}

void World::randomizeTextures() {
    // TODO: Implement texture randomization
}

void World::generateMaze() {
    // TODO: Implement maze generation algorithm
    addRandomObstacles(20, 15.0f, 30.0f);
}

void World::generateCorridor() {
    // TODO: Implement corridor generation
    addRandomObstacles(15, 10.0f, 25.0f);
}

void World::generateOpenField() {
    // TODO: Implement open field generation
    addRandomObstacles(10, 20.0f, 40.0f);
}

void World::generateObstacleCourse() {
    // TODO: Implement obstacle course generation
    addRandomObstacles(25, 8.0f, 20.0f);
    addMovingObstacles(5, 12.0f, 18.0f, 50.0f);
}

void World::updateMovingObstacles(float dt) {
    for (auto& obs : obstacles_) {
        if (obs.is_moving) {
            obs.position += obs.velocity * dt;
        }
    }
}

void World::bounceObstaclesOffWalls() {
    for (auto& obs : obstacles_) {
        if (obs.is_moving) {
            if (obs.position.x - obs.radius < 0 || obs.position.x + obs.radius >= width_) {
                obs.velocity.x = -obs.velocity.x;
            }
            if (obs.position.y - obs.radius < 0 || obs.position.y + obs.radius >= height_) {
                obs.velocity.y = -obs.velocity.y;
            }
        }
    }
}

} // namespace sim
