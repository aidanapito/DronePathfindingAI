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
    addSkyscrapers(12);
    
    // Add ground obstacles
    addGroundObstacles(15);
    
    // Add mountain pass
    addMountainPass();
    
    // Add new structure types
    addBridges(3);
    addTunnels(2);
    addArches(4);
    addPyramids(2);
    addSphereBuildings(3);
    addWindTurbines(5);
    addRadioTowers(2);
    addWaterTowers(3);
    addFactories(4);
    
    // Add special landmarks
    addLandmarks();
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

void World::addBridges(int count) {
    std::uniform_real_distribution<float> x_dist(100.0f, width_ - 100.0f);
    std::uniform_real_distribution<float> y_dist(100.0f, height_ - 100.0f);
    std::uniform_real_distribution<float> rot_dist(0.0f, 2.0f * M_PI);
    
    for (int i = 0; i < count; ++i) {
        float x = x_dist(rng_);
        float y = y_dist(rng_);
        float rotation = rot_dist(rng_);
        generateBridge(x, y, rotation);
    }
}

void World::addTunnels(int count) {
    std::uniform_real_distribution<float> x_dist(150.0f, width_ - 150.0f);
    std::uniform_real_distribution<float> y_dist(150.0f, height_ - 150.0f);
    std::uniform_real_distribution<float> rot_dist(0.0f, 2.0f * M_PI);
    
    for (int i = 0; i < count; ++i) {
        float x = x_dist(rng_);
        float y = y_dist(rng_);
        float rotation = rot_dist(rng_);
        generateTunnel(x, y, rotation);
    }
}

void World::addArches(int count) {
    std::uniform_real_distribution<float> x_dist(80.0f, width_ - 80.0f);
    std::uniform_real_distribution<float> y_dist(80.0f, height_ - 80.0f);
    std::uniform_real_distribution<float> rot_dist(0.0f, 2.0f * M_PI);
    
    for (int i = 0; i < count; ++i) {
        float x = x_dist(rng_);
        float y = y_dist(rng_);
        float rotation = rot_dist(rng_);
        generateArch(x, y, rotation);
    }
}

void World::addPyramids(int count) {
    std::uniform_real_distribution<float> x_dist(100.0f, width_ - 100.0f);
    std::uniform_real_distribution<float> y_dist(100.0f, height_ - 100.0f);
    
    for (int i = 0; i < count; ++i) {
        float x = x_dist(rng_);
        float y = y_dist(rng_);
        generatePyramid(x, y);
    }
}

void World::addSphereBuildings(int count) {
    std::uniform_real_distribution<float> x_dist(120.0f, width_ - 120.0f);
    std::uniform_real_distribution<float> y_dist(120.0f, height_ - 120.0f);
    
    for (int i = 0; i < count; ++i) {
        float x = x_dist(rng_);
        float y = y_dist(rng_);
        generateSphereBuilding(x, y);
    }
}

void World::addWindTurbines(int count) {
    std::uniform_real_distribution<float> x_dist(60.0f, width_ - 60.0f);
    std::uniform_real_distribution<float> y_dist(60.0f, height_ - 60.0f);
    
    for (int i = 0; i < count; ++i) {
        float x = x_dist(rng_);
        float y = y_dist(rng_);
        generateWindTurbine(x, y);
    }
}

void World::addRadioTowers(int count) {
    std::uniform_real_distribution<float> x_dist(80.0f, width_ - 80.0f);
    std::uniform_real_distribution<float> y_dist(80.0f, height_ - 80.0f);
    
    for (int i = 0; i < count; ++i) {
        float x = x_dist(rng_);
        float y = y_dist(rng_);
        generateRadioTower(x, y);
    }
}

void World::addWaterTowers(int count) {
    std::uniform_real_distribution<float> x_dist(70.0f, width_ - 70.0f);
    std::uniform_real_distribution<float> y_dist(70.0f, height_ - 70.0f);
    
    for (int i = 0; i < count; ++i) {
        float x = x_dist(rng_);
        float y = y_dist(rng_);
        generateWaterTower(x, y);
    }
}

void World::addFactories(int count) {
    std::uniform_real_distribution<float> x_dist(150.0f, width_ - 150.0f);
    std::uniform_real_distribution<float> y_dist(150.0f, height_ - 150.0f);
    std::uniform_real_distribution<float> rot_dist(0.0f, 2.0f * M_PI);
    
    for (int i = 0; i < count; ++i) {
        float x = x_dist(rng_);
        float y = y_dist(rng_);
        float rotation = rot_dist(rng_);
        generateFactory(x, y, rotation);
    }
}

void World::generateSkyscraper(float x, float y) {
    Obstacle skyscraper;
    skyscraper.x = x;
    skyscraper.y = y;
    skyscraper.z = 50.0f;  // Center height above ground
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
    obstacle.z = 10.0f;  // Half height above ground
    obstacle.radius = 15.0f;
    obstacle.height = 20.0f;
    obstacle.is_vertical = false;
    obstacle.type = ObstacleType::GROUND_OBSTACLE;
    obstacles_.push_back(obstacle);
}

void World::generateBridge(float x, float y, float rotation) {
    Obstacle bridge;
    bridge.x = x;
    bridge.y = y;
    bridge.z = 20.0f;  // Bridge deck height above ground
    bridge.radius = 25.0f;
    bridge.height = 20.0f;
    bridge.is_vertical = false;
    bridge.type = ObstacleType::BRIDGE;
    bridge.rotation = rotation;
    bridge.width = 120.0f;  // Bridge length
    bridge.depth = 20.0f;   // Bridge width
    obstacles_.push_back(bridge);
}

void World::generateTunnel(float x, float y, float rotation) {
    Obstacle tunnel;
    tunnel.x = x;
    tunnel.y = y;
    tunnel.z = 15.0f;  // Tunnel entrance height above ground
    tunnel.radius = 20.0f;
    tunnel.height = 30.0f;
    tunnel.is_vertical = false;
    tunnel.type = ObstacleType::TUNNEL;
    tunnel.rotation = rotation;
    tunnel.width = 80.0f;   // Tunnel length
    tunnel.depth = 40.0f;   // Tunnel width
    obstacles_.push_back(tunnel);
}

void World::generateArch(float x, float y, float rotation) {
    Obstacle arch;
    arch.x = x;
    arch.y = y;
    arch.z = 25.0f;  // Arch center height above ground
    arch.radius = 15.0f;
    arch.height = 50.0f;
    arch.is_vertical = false;
    arch.type = ObstacleType::ARCH;
    arch.rotation = rotation;
    arch.width = 60.0f;   // Arch span
    arch.depth = 10.0f;   // Arch thickness
    obstacles_.push_back(arch);
}

void World::generatePyramid(float x, float y) {
    Obstacle pyramid;
    pyramid.x = x;
    pyramid.y = y;
    pyramid.z = 40.0f;  // Pyramid center height above ground
    pyramid.radius = 30.0f;
    pyramid.height = 80.0f;
    pyramid.is_vertical = false;
    pyramid.type = ObstacleType::PYRAMID;
    pyramid.rotation = 0.0f;
    pyramid.width = 60.0f;   // Base width
    pyramid.depth = 60.0f;   // Base depth
    obstacles_.push_back(pyramid);
}

void World::generateSphereBuilding(float x, float y) {
    Obstacle sphere;
    sphere.x = x;
    sphere.y = y;
    sphere.z = 35.0f;  // Sphere center height above ground
    sphere.radius = 25.0f;
    sphere.height = 70.0f;
    sphere.is_vertical = true;
    sphere.type = ObstacleType::SPHERE_BUILDING;
    sphere.rotation = 0.0f;
    sphere.width = 50.0f;   // Sphere diameter
    sphere.depth = 50.0f;   // Sphere diameter
    obstacles_.push_back(sphere);
}

void World::generateWindTurbine(float x, float y) {
    Obstacle turbine;
    turbine.x = x;
    turbine.y = y;
    turbine.z = 60.0f;  // Turbine hub height above ground
    turbine.radius = 5.0f;
    turbine.height = 120.0f;
    turbine.is_vertical = true;
    turbine.type = ObstacleType::WIND_TURBINE;
    turbine.rotation = 0.0f;
    turbine.width = 10.0f;   // Tower width
    turbine.depth = 10.0f;   // Tower depth
    obstacles_.push_back(turbine);
}

void World::generateRadioTower(float x, float y) {
    Obstacle tower;
    tower.x = x;
    tower.y = y;
    tower.z = 80.0f;  // Tower center height above ground
    tower.radius = 8.0f;
    tower.height = 160.0f;
    tower.is_vertical = true;
    tower.type = ObstacleType::RADIO_TOWER;
    tower.rotation = 0.0f;
    tower.width = 16.0f;   // Tower width
    tower.depth = 16.0f;   // Tower depth
    obstacles_.push_back(tower);
}

void World::generateWaterTower(float x, float y) {
    Obstacle waterTower;
    waterTower.x = x;
    waterTower.y = y;
    waterTower.z = 45.0f;  // Tank center height above ground
    waterTower.radius = 20.0f;
    waterTower.height = 90.0f;
    waterTower.is_vertical = true;
    waterTower.type = ObstacleType::WATER_TOWER;
    waterTower.rotation = 0.0f;
    waterTower.width = 40.0f;   // Tank diameter
    waterTower.depth = 40.0f;   // Tank diameter
    obstacles_.push_back(waterTower);
}

void World::generateFactory(float x, float y, float rotation) {
    Obstacle factory;
    factory.x = x;
    factory.y = y;
    factory.z = 25.0f;  // Factory center height above ground
    factory.radius = 35.0f;
    factory.height = 50.0f;
    factory.is_vertical = true;
    factory.type = ObstacleType::FACTORY;
    factory.rotation = rotation;
    factory.width = 140.0f;  // Factory length
    factory.depth = 70.0f;   // Factory width
    obstacles_.push_back(factory);
}

void World::addLandmarks() {
    // Add a few special landmark structures at strategic locations
    generateLandmark(width_ * 0.25f, height_ * 0.25f, 0); // Observatory
    generateLandmark(width_ * 0.75f, height_ * 0.75f, 1); // Monument
    generateLandmark(width_ * 0.5f, height_ * 0.5f, 2);   // Central Tower
}

void World::generateLandmark(float x, float y, int landmarkType) {
    Obstacle landmark;
    landmark.x = x;
    landmark.y = y;
    landmark.rotation = 0.0f;
    
    switch (landmarkType) {
        case 0: // Observatory
            landmark.z = 60.0f;
            landmark.radius = 30.0f;
            landmark.height = 120.0f;
            landmark.is_vertical = true;
            landmark.type = ObstacleType::SPHERE_BUILDING;
            landmark.width = 60.0f;
            landmark.depth = 60.0f;
            break;
        case 1: // Monument
            landmark.z = 40.0f;
            landmark.radius = 20.0f;
            landmark.height = 80.0f;
            landmark.is_vertical = true;
            landmark.type = ObstacleType::RADIO_TOWER;
            landmark.width = 40.0f;
            landmark.depth = 40.0f;
            break;
        case 2: // Central Tower
            landmark.z = 100.0f;
            landmark.radius = 25.0f;
            landmark.height = 200.0f;
            landmark.is_vertical = true;
            landmark.type = ObstacleType::SKYSCRAPER;
            landmark.width = 50.0f;
            landmark.depth = 50.0f;
            break;
    }
    
    obstacles_.push_back(landmark);
}

void World::generateMountain(float x, float y, float height) {
    Obstacle mountain;
    mountain.x = x;
    mountain.y = y;
    mountain.z = height/2.0f;  // Center at half height above ground
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
