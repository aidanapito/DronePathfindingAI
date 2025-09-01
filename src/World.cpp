#include "World.h"
#include <cmath>
#include <iostream>

World::World(int width, int height, int depth) 
    : width_(width), height_(height), depth_(depth), target_building_(nullptr) {
    rng_.seed(std::random_device{}());
}

void World::generateTerrain() {
    // Clear existing obstacles
    obstacles_.clear();
    target_building_ = nullptr;
    
    // Create a city-like environment with dense skyscrapers
    // This creates streets between the buildings
    
    // City grid parameters
    float cityWidth = width_ * 0.8f;  // Use 80% of world width
    float cityHeight = height_ * 0.8f; // Use 80% of world height
    float blockSize = cityWidth / 4.0f; // 4 blocks across
    float buildingSpacing = blockSize / 3.0f; // 3 buildings per block
    
    // Starting position for the grid
    float startX = (width_ - cityWidth) / 2.0f;
    float startY = (height_ - cityHeight) / 2.0f;
    
    // Generate dense city grid with multiple buildings per block
    for (int blockRow = 0; blockRow < 4; ++blockRow) {
        for (int blockCol = 0; blockCol < 4; ++blockCol) {
            // For each block, add multiple buildings
            for (int buildingRow = 0; buildingRow < 3; ++buildingRow) {
                for (int buildingCol = 0; buildingCol < 3; ++buildingCol) {
                    // Calculate building position within the block
                    float blockX = startX + blockCol * blockSize;
                    float blockY = startY + blockRow * blockSize;
                    float x = blockX + (buildingCol + 0.5f) * buildingSpacing;
                    float y = blockY + (buildingRow + 0.5f) * buildingSpacing;
                    
                    // Generate random height for variety (40-180 units)
                    float buildingHeight = 40.0f + (rng_() % 140);
                    
                    // Create skyscraper with custom height
                    generateCitySkyscraper(x, y, buildingHeight);
                }
            }
        }
    }
    
    // Add a few wind turbines outside the city
    addWindTurbines(2);
    
    // Generate random target building
    generateRandomTargetBuilding();
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
    // Create a simple mountain range along one side
    for (int i = 0; i < 4; ++i) {
        float x = 100.0f + i * 80.0f;
        float y = 200.0f;
        float height = 60.0f + (rng_() % 20);
        generateMountain(x, y, height);
    }
}

void World::addBridges(int count) {
    // Place bridges in strategic locations
    generateBridge(width_ * 0.5f, height_ * 0.5f, 0.0f); // Center bridge
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
    // Place arches in strategic locations
    generateArch(width_ * 0.3f, height_ * 0.7f, 0.0f); // Left arch
    generateArch(width_ * 0.7f, height_ * 0.3f, 1.57f); // Right arch (rotated)
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
    // Place wind turbines in strategic locations
    generateWindTurbine(width_ * 0.2f, height_ * 0.8f); // Top left
    generateWindTurbine(width_ * 0.8f, height_ * 0.2f); // Bottom right
}

void World::addWaterTowers(int count) {
    // Place water tower in strategic location
    generateWaterTower(width_ * 0.6f, height_ * 0.6f); // Center-right
}

void World::addFactories(int count) {
    // Not used in simplified world
}

void World::addLandmarks() {
    // Not used in simplified world
}

void World::generateLandmark(float x, float y, int landmarkType) {
    // Not used in simplified world
}

void World::generateSkyscraper(float x, float y) {
    generateCitySkyscraper(x, y, 100.0f); // Default height
}

void World::generateCitySkyscraper(float x, float y, float height) {
    Obstacle skyscraper;
    skyscraper.x = x;
    skyscraper.y = y;
    skyscraper.z = height / 2.0f;  // Center height above ground
    skyscraper.radius = 15.0f;     // Smaller radius for denser city
    skyscraper.height = height;
    skyscraper.is_vertical = true; // Ensure it's vertical
    skyscraper.type = ObstacleType::SKYSCRAPER;
    skyscraper.rotation = 0.0f;
    skyscraper.width = 30.0f;   // Building width
    skyscraper.depth = 30.0f;   // Building depth
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
        if (obs.type == ObstacleType::SKYSCRAPER) {
            // For skyscrapers, use rectangular collision detection
            float halfWidth = obs.width / 2.0f;
            float halfDepth = obs.depth / 2.0f;
            float halfHeight = obs.height / 2.0f;
            
            // Check if drone is within the building's rectangular bounds
            bool inX = (x >= obs.x - halfWidth - radius) && (x <= obs.x + halfWidth + radius);
            bool inY = (y >= obs.y - halfDepth - radius) && (y <= obs.y + halfDepth + radius);
            bool inZ = (z >= obs.z - halfHeight - radius) && (z <= obs.z + halfHeight + radius);
            
            if (inX && inY && inZ) {
                return true; // Collision detected
            }
        } else {
            // For other objects, use circular collision detection
            float dx = x - obs.x;
            float dy = y - obs.y;
            float dz = z - obs.z;
            float distance = sqrt(dx*dx + dy*dy + dz*dz);
            
            if (distance < (radius + obs.radius)) {
                return true; // Collision detected
            }
        }
    }
    return false;
}

float World::getGroundHeight(float x, float y) const {
    // Simple ground height - could be enhanced with terrain generation
    return 0.0f;
}

void World::generateTargetBuilding(float x, float y) {
    Obstacle target;
    target.x = x;
    target.y = y;
    target.z = 30.0f;  // Target building center height above ground
    target.radius = 25.0f;
    target.height = 60.0f;
    target.is_vertical = true;
    target.type = ObstacleType::FACTORY; // Use factory type for green rendering
    target.rotation = 0.0f;
    target.width = 50.0f;   // Building width
    target.depth = 50.0f;   // Building depth
    obstacles_.push_back(target);
    
    // Store pointer to target building
    target_building_ = &obstacles_.back();
}

void World::generateRandomTargetBuilding() {
    // Generate target building at a random location within the city area
    std::uniform_real_distribution<float> x_dist(200.0f, width_ - 200.0f);
    std::uniform_real_distribution<float> y_dist(200.0f, height_ - 200.0f);
    
    float x = x_dist(rng_);
    float y = y_dist(rng_);
    
    generateTargetBuilding(x, y);
    
    std::cout << "🎯 Target building generated at: (" << x << ", " << y << ", " << (30.0f + 60.0f) << ")" << std::endl;
}
