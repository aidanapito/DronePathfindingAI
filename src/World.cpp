#include "World.h"
#include <cmath>

World::World(int width, int height, int depth) 
    : width_(width), height_(height), depth_(depth) {
    rng_.seed(std::random_device{}());
}

void World::generateTerrain() {
    // Clear existing obstacles
    obstacles_.clear();
    
    // Add skyscrapers
    addSkyscrapers(15);
    
    // Add ground obstacles
    addGroundObstacles(20);
    
    // Add mountain pass
    addMountainPass();
}

void World::addSkyscrapers(int count) {
    std::uniform_real_distribution<float> x_dist(50.0f, width_ - 50.0f);
    std::uniform_real_distribution<float> y_dist(50.0f, height_ - 50.0f);
    
    for (int i = 0; i < count; ++i) {
        float x = x_dist(rng_);
        float y = y_dist(rng_);
        generateSkyscraper(x, y);
    }
}

void World::addGroundObstacles(int count) {
    std::uniform_real_distribution<float> x_dist(50.0f, width_ - 50.0f);
    std::uniform_real_distribution<float> y_dist(50.0f, height_ - 50.0f);
    
    for (int i = 0; i < count; ++i) {
        float x = x_dist(rng_);
        float y = y_dist(rng_);
        generateGroundObstacle(x, y);
    }
}

void World::addMountainPass() {
    // Create a mountain range
    for (int i = 0; i < 8; ++i) {
        float x = 200.0f + i * 100.0f;
        float y = 300.0f + (i % 2) * 50.0f;
        float height = 80.0f + (rng_() % 40);
        generateMountain(x, y, height);
    }
}

void World::generateSkyscraper(float x, float y) {
    Obstacle skyscraper;
    skyscraper.x = x;
    skyscraper.y = y;
    skyscraper.z = 50.0f;  // Base height 50
    skyscraper.radius = 20.0f;
    skyscraper.height = 100.0f;
    skyscraper.is_vertical = true;
    skyscraper.type = ObstacleType::SKYSCRAPER;
    obstacles_.push_back(skyscraper);
}

void World::generateGroundObstacle(float x, float y) {
    Obstacle obstacle;
    obstacle.x = x;
    obstacle.y = y;
    obstacle.z = 25.0f;  // Half height for ground
    obstacle.radius = 15.0f;
    obstacle.height = 20.0f;
    obstacle.is_vertical = false;
    obstacle.type = ObstacleType::GROUND_OBSTACLE;
    obstacles_.push_back(obstacle);
}

void World::generateMountain(float x, float y, float height) {
    Obstacle mountain;
    mountain.x = x;
    mountain.y = y;
    mountain.z = height/2.0f;  // Center at half height
    mountain.radius = 40.0f;
    mountain.height = height;
    mountain.is_vertical = false;
    mountain.type = ObstacleType::MOUNTAIN;
    obstacles_.push_back(mountain);
}

bool World::checkCollision(float x, float y, float z, float radius) const {
    for (const auto& obs : obstacles_) {
        float dx = x - obs.x;
        float dy = y - obs.y;
        float dz = z - obs.z;
        float distance = sqrt(dx*dx + dy*dy + dz*dz);
        
        if (distance < (radius + obs.radius)) {
            return true; // Collision detected
        }
    }
    return false;
}

float World::getGroundHeight(float x, float y) const {
    // Simple ground height - could be enhanced with terrain generation
    return 0.0f;
}
