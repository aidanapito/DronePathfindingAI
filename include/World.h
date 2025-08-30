#pragma once

#include <vector>
#include <random>

enum class ObstacleType {
    SKYSCRAPER,
    GROUND_OBSTACLE,
    MOUNTAIN
};

struct Obstacle {
    float x, y, z;  // Position
    float radius;
    float height;
    bool is_vertical;  // true for buildings, false for ground obstacles
    ObstacleType type;  // Type of obstacle for 3D rendering
};

class World {
public:
    World(int width = 1200, int height = 800, int depth = 600);
    
    // World generation
    void generateTerrain();
    void addSkyscrapers(int count);
    void addGroundObstacles(int count);
    void addMountainPass();
    
    // Getters
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    int getDepth() const { return depth_; }
    const std::vector<Obstacle>& getObstacles() const { return obstacles_; }
    
    // Collision detection
    bool checkCollision(float x, float y, float z, float radius) const;
    float getGroundHeight(float x, float y) const;
    
private:
    int width_, height_, depth_;
    std::vector<Obstacle> obstacles_;
    std::mt19937 rng_;
    
    // Terrain generation helpers
    void generateSkyscraper(float x, float y);
    void generateGroundObstacle(float x, float y);
    void generateMountain(float x, float y, float height);
};
