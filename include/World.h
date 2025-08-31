#pragma once

#include <vector>
#include <random>

enum class ObstacleType {
    SKYSCRAPER,
    GROUND_OBSTACLE,
    MOUNTAIN,
    BRIDGE,
    TUNNEL,
    ARCH,
    PYRAMID,
    SPHERE_BUILDING,
    WIND_TURBINE,
    RADIO_TOWER,
    WATER_TOWER,
    FACTORY
};

struct Obstacle {
    float x, y, z;  // Position
    float radius;
    float height;
    bool is_vertical;  // true for buildings, false for ground obstacles
    ObstacleType type;  // Type of obstacle for 3D rendering
    float rotation;     // Rotation around Y axis for directional objects
    float width;        // Width for non-circular objects
    float depth;        // Depth for non-circular objects
};

class World {
public:
    World(int width = 1200, int height = 800, int depth = 600);
    
    // World generation
    void generateTerrain();
    void addSkyscrapers(int count);
    void addGroundObstacles(int count);
    void addMountainPass();
    void addBridges(int count);
    void addTunnels(int count);
    void addArches(int count);
    void addPyramids(int count);
    void addSphereBuildings(int count);
    void addWindTurbines(int count);
    void addRadioTowers(int count);
    void addWaterTowers(int count);
    void addFactories(int count);
    void addLandmarks();
    
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
    void generateCitySkyscraper(float x, float y, float height);
    void generateGroundObstacle(float x, float y);
    void generateMountain(float x, float y, float height);
    void generateBridge(float x, float y, float rotation);
    void generateTunnel(float x, float y, float rotation);
    void generateArch(float x, float y, float rotation);
    void generatePyramid(float x, float y);
    void generateSphereBuilding(float x, float y);
    void generateWindTurbine(float x, float y);
    void generateRadioTower(float x, float y);
    void generateWaterTower(float x, float y);
    void generateFactory(float x, float y, float rotation);
    void generateLandmark(float x, float y, int landmarkType);
};
