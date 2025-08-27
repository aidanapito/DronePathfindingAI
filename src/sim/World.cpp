#include "sim/World.h"
#include <iostream>
#include <algorithm>
#include <cmath>

namespace sim {

// 2D PathNode struct for backward compatibility
struct PathNode {
    int x, y;
    float g_cost;  // Cost from start to current node
    float h_cost;  // Heuristic cost from current node to goal
    float f_cost;  // Total cost (g + h)
    std::pair<int, int> parent;  // Parent node coordinates
    
    PathNode(int x, int y, float g, float h, 
             std::pair<int, int> parent = {-1, -1})
        : x(x), y(y), g_cost(g), h_cost(h), f_cost(g + h), parent(parent) {}
    
    bool operator>(const PathNode& other) const {
        return f_cost > other.f_cost;
    }
};

World::World(int width, int height, int depth) 
    : width_(width), height_(height), depth_(depth),
      lighting_brightness_(1.0f), lighting_contrast_(1.0f),
      ground_level_(depth / 2.0f), max_building_height_(depth * 0.8f),
      terrain_variation_(depth * 0.1f) {
    // Initialize random number generator
    rng_.seed(std::random_device{}());
    
    // Initialize start and goal positions in 3D
    start_position_ = cv::Point3f(50.0f, height_ / 2.0f, ground_level_);
    goal_position_ = cv::Point3f(width_ - 50.0f, height_ / 2.0f, ground_level_);
    
    // Initialize occupancy grids
    occupancy_grid_2d_.resize(height_, std::vector<bool>(width_, false));
    occupancy_grid_3d_.resize(depth_, std::vector<std::vector<bool>>(height_, std::vector<bool>(width_, false)));
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
        case MapType::SKYSCRAPER:
            generateSkyscraper();
            break;
        case MapType::UNDERWATER:
            generateUnderwater();
            break;
        case MapType::MOUNTAIN_PASS:
            generateMountainPass();
            break;
    }
}

void World::addRandomObstacles(int count, float min_radius, float max_radius, 
                               float min_height, float max_height) {
    std::uniform_real_distribution<float> radius_dist(min_radius, max_radius);
    std::uniform_real_distribution<float> x_dist(100.0f, width_ - 100.0f);
    std::uniform_real_distribution<float> y_dist(100.0f, height_ - 100.0f);
    std::uniform_real_distribution<float> z_dist(ground_level_, max_building_height_);
    std::uniform_real_distribution<float> height_dist(min_height, max_height);
    
    for (int i = 0; i < count; ++i) {
        Obstacle obs;
        obs.position = cv::Point3f(x_dist(rng_), y_dist(rng_), z_dist(rng_));
        obs.radius = radius_dist(rng_);
        obs.height = height_dist(rng_);
        obs.is_moving = false;
        obs.velocity = cv::Point3f(0.0f, 0.0f, 0.0f);
        obs.max_speed = 0.0f;
        obs.is_vertical = false;
        
        obstacles_.push_back(obs);
    }
}

void World::addMovingObstacles(int count, float min_radius, float max_radius, 
                               float max_speed, float min_height, float max_height) {
    std::uniform_real_distribution<float> radius_dist(min_radius, max_radius);
    std::uniform_real_distribution<float> x_dist(100.0f, width_ - 100.0f);
    std::uniform_real_distribution<float> y_dist(100.0f, height_ - 100.0f);
    std::uniform_real_distribution<float> z_dist(ground_level_, max_building_height_);
    std::uniform_real_distribution<float> height_dist(min_height, max_height);
    std::uniform_real_distribution<float> angle_dist(0.0f, 2.0f * M_PI);
    
    for (int i = 0; i < count; ++i) {
        Obstacle obs;
        obs.position = cv::Point3f(x_dist(rng_), y_dist(rng_), z_dist(rng_));
        obs.radius = radius_dist(rng_);
        obs.height = height_dist(rng_);
        obs.is_moving = true;
        
        float angle = angle_dist(rng_);
        float speed = max_speed * (0.5f + 0.5f * (rng_() % 100) / 100.0f);
        obs.velocity = cv::Point3f(speed * cos(angle), speed * sin(angle), 0.0f);
        obs.max_speed = max_speed;
        obs.is_vertical = false;
        
        obstacles_.push_back(obs);
    }
}

void World::addVerticalObstacles(int count, float min_radius, float max_radius, 
                                 float min_height, float max_height) {
    std::uniform_real_distribution<float> radius_dist(min_radius, max_radius);
    std::uniform_real_distribution<float> x_dist(100.0f, width_ - 100.0f);
    std::uniform_real_distribution<float> y_dist(100.0f, height_ - 100.0f);
    std::uniform_real_distribution<float> height_dist(min_height, max_height);
    
    for (int i = 0; i < count; ++i) {
        Obstacle obs;
        obs.position = cv::Point3f(x_dist(rng_), y_dist(rng_), ground_level_);
        obs.radius = radius_dist(rng_);
        obs.height = height_dist(rng_);
        obs.is_moving = false;
        obs.velocity = cv::Point3f(0.0f, 0.0f, 0.0f);
        obs.max_speed = 0.0f;
        obs.is_vertical = true;
        
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
        // Convert 3D position to 2D for rendering
        cv::Point2f pos_2d(obs.position.x, obs.position.y);
        cv::circle(output, pos_2d, obs.radius, cv::Scalar(0, 0, 0), -1);
    }
    
    // Draw start and goal (convert 3D to 2D)
    cv::Point2f start_2d(start_position_.x, start_position_.y);
    cv::Point2f goal_2d(goal_position_.x, goal_position_.y);
    cv::circle(output, start_2d, 10, cv::Scalar(0, 255, 0), -1);
    cv::circle(output, goal_2d, 10, cv::Scalar(0, 0, 255), -1);
}

bool World::checkCollision(const cv::Point2f& position, float radius) const {
    for (const auto& obs : obstacles_) {
        float distance = cv::norm(position - cv::Point2f(obs.position.x, obs.position.y));
        if (distance < (radius + obs.radius)) {
            return true;
        }
    }
    return false;
}

bool World::checkCollision(const cv::Point3f& position, float radius) const {
    for (const auto& obs : obstacles_) {
        float distance = cv::norm(position - obs.position);
        if (distance < (radius + obs.radius)) {
            // Check if obstacle extends to this height
            if (obs.is_vertical) {
                if (position.z >= obs.position.z && position.z <= obs.position.z + obs.height) {
                    return true;
                }
            } else {
                return true;
            }
        }
    }
    return false;
}

bool World::isInBounds(const cv::Point2f& position) const {
    return position.x >= 0 && position.x < width_ && 
           position.y >= 0 && position.y < height_;
}

bool World::isInBounds(const cv::Point3f& position) const {
    return position.x >= 0 && position.x < width_ && 
           position.y >= 0 && position.y < height_ &&
           position.z >= 0 && position.z < depth_;
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
    // Create a more navigable maze with corridors
    obstacles_.clear();
    
    // Add some walls to create corridors
    for (int i = 0; i < 10; ++i) {
        // Horizontal walls
        float y = 100.0f + i * 50.0f;
        for (int j = 0; j < 8; ++j) {
            float x = 150.0f + j * 80.0f;
            Obstacle obs;
            obs.position = cv::Point3f(x, y, ground_level_);
            obs.radius = 8.0f;
            obs.height = 10.0f;
            obs.is_moving = false;
            obs.velocity = cv::Point3f(0.0f, 0.0f, 0.0f);
            obs.max_speed = 0.0f;
            obs.is_vertical = false;
            obstacles_.push_back(obs);
        }
    }
    
    // Add some random obstacles but not too many
    addRandomObstacles(15, 10.0f, 20.0f);
}

void World::generateCorridor() {
    // Create a clear corridor with minimal obstacles
    obstacles_.clear();
    
    // Add some walls to create a clear path
    for (int i = 0; i < 5; ++i) {
        // Left wall
        float x = 200.0f;
        float y = 100.0f + i * 100.0f;
        Obstacle obs;
        obs.position = cv::Point3f(x, y, ground_level_);
        obs.radius = 12.0f;
        obs.height = 10.0f;
        obs.is_moving = false;
        obs.velocity = cv::Point3f(0.0f, 0.0f, 0.0f);
        obs.max_speed = 0.0f;
        obs.is_vertical = false;
        obstacles_.push_back(obs);
        
        // Right wall
        x = 600.0f;
        obs.position = cv::Point3f(x, y, ground_level_);
        obstacles_.push_back(obs);
    }
    
    // Add very few random obstacles in the corridor
    addRandomObstacles(5, 8.0f, 15.0f);
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

void World::generateSkyscraper() {
    obstacles_.clear();
    
    // Create a city-like environment with tall buildings
    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 6; ++j) {
            if (i == 0 && j == 2) continue; // Leave space for start
            if (i == 7 && j == 2) continue; // Leave space for goal
            
            float x = 100.0f + i * 80.0f;
            float y = 100.0f + j * 80.0f;
            float height = 50.0f + (rng_() % 100) * 0.5f; // Random building height
            
            Obstacle obs;
            obs.position = cv::Point3f(x, y, ground_level_);
            obs.radius = 8.0f; // Much smaller radius for realistic buildings
            obs.height = height;
            obs.is_moving = false;
            obs.velocity = cv::Point3f(0.0f, 0.0f, 0.0f);
            obs.max_speed = 0.0f;
            obs.is_vertical = true;
            obstacles_.push_back(obs);
        }
    }
    
    // Add some flying obstacles at different altitudes
    addRandomObstacles(10, 15.0f, 30.0f, 50.0f, 150.0f);
}

void World::generateUnderwater() {
    obstacles_.clear();
    
    // Create underwater environment with coral reefs and sea creatures
    for (int i = 0; i < 12; ++i) {
        float x = 100.0f + (rng_() % 600);
        float y = 100.0f + (rng_() % 400);
        float height = 20.0f + (rng_() % 60);
        
        Obstacle obs;
        obs.position = cv::Point3f(x, y, ground_level_);
        obs.radius = 15.0f + (rng_() % 25);
        obs.height = height;
        obs.is_moving = false;
        obs.velocity = cv::Point3f(0.0f, 0.0f, 0.0f);
        obs.max_speed = 0.0f;
        obs.is_vertical = true;
        obstacles_.push_back(obs);
    }
    
    // Add moving sea creatures
    addMovingObstacles(8, 20.0f, 35.0f, 30.0f, 30.0f, 80.0f);
}

void World::generateMountainPass() {
    obstacles_.clear();
    
    // Create mountainous terrain with varying elevations
    for (int i = 0; i < 15; ++i) {
        float x = 100.0f + (rng_() % 600);
        float y = 100.0f + (rng_() % 400);
        float height = 40.0f + (rng_() % 120);
        
        Obstacle obs;
        obs.position = cv::Point3f(x, y, ground_level_);
        obs.radius = 20.0f + (rng_() % 30);
        obs.height = height;
        obs.is_moving = false;
        obs.velocity = cv::Point3f(0.0f, 0.0f, 0.0f);
        obs.max_speed = 0.0f;
        obs.is_vertical = true;
        obstacles_.push_back(obs);
    }
    
    // Create a pass through the mountains
    for (int i = 0; i < 5; ++i) {
        float y = 250.0f + i * 20.0f;
        for (int j = 0; j < 4; ++j) {
            float x = 300.0f + j * 50.0f;
            
            Obstacle obs;
            obs.position = cv::Point3f(x, y, ground_level_);
            obs.radius = 8.0f;
            obs.height = 30.0f;
            obs.is_moving = false;
            obs.velocity = cv::Point3f(0.0f, 0.0f, 0.0f);
            obs.max_speed = 0.0f;
            obs.is_vertical = true;
            obstacles_.push_back(obs);
        }
    }
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
            // Bounce off 2D boundaries (x, y)
            if (obs.position.x - obs.radius < 0 || obs.position.x + obs.radius >= width_) {
                obs.velocity.x = -obs.velocity.x;
            }
            if (obs.position.y - obs.radius < 0 || obs.position.y + obs.radius >= height_) {
                obs.velocity.y = -obs.velocity.y;
            }
            
            // Bounce off 3D boundaries (z)
            if (obs.position.z - obs.radius < 0 || obs.position.z + obs.radius >= depth_) {
                obs.velocity.z = -obs.velocity.z;
            }
        }
    }
}

// Pathfinding methods
std::vector<cv::Point2f> World::findPathAStar(const cv::Point2f& start, const cv::Point2f& goal, float grid_size) {
    // Convert world coordinates to grid coordinates
    int grid_width = static_cast<int>(width_ / grid_size);
    int grid_height = static_cast<int>(height_ / grid_size);
    
    int start_x = static_cast<int>(start.x / grid_size);
    int start_y = static_cast<int>(start.y / grid_size);
    int goal_x = static_cast<int>(goal.x / grid_size);
    int goal_y = static_cast<int>(goal.y / grid_size);
    
    // Validate grid positions
    if (!isValidGridPosition(start_x, start_y, grid_width, grid_height) ||
        !isValidGridPosition(goal_x, goal_y, grid_width, grid_height)) {
        return {};
    }
    
    // Priority queue for A* algorithm
    std::priority_queue<PathNode, std::vector<PathNode>, std::greater<PathNode>> open_set;
    std::unordered_set<std::string> closed_set;
    
    // Initialize start node
    PathNode start_node(start_x, start_y, 0.0f, calculateHeuristic(start_x, start_y, goal_x, goal_y), {-1, -1});
    open_set.push(start_node);
    
    // Parent tracking for path reconstruction
    std::vector<std::vector<std::pair<int, int>>> parent(grid_height, 
        std::vector<std::pair<int, int>>(grid_width, {-1, -1}));
    
    // Cost tracking
    std::vector<std::vector<float>> g_cost(grid_height, std::vector<float>(grid_width, std::numeric_limits<float>::max()));
    g_cost[start_y][start_x] = 0.0f;
    
    while (!open_set.empty()) {
        PathNode current = open_set.top();
        open_set.pop();
        
        // Create unique key for closed set
        std::string key = std::to_string(current.x) + "," + std::to_string(current.y);
        if (closed_set.find(key) != closed_set.end()) {
            continue;
        }
        closed_set.insert(key);
        
        // Check if we reached the goal
        if (current.x == goal_x && current.y == goal_y) {
            // Reconstruct path
            std::vector<cv::Point2f> path;
            int x = goal_x, y = goal_y;
            
            while (x != start_x || y != start_y) {
                path.push_back(cv::Point2f(x * grid_size + grid_size/2, y * grid_size + grid_size/2));
                int temp_x = parent[y][x].first;
                int temp_y = parent[y][x].second;
                x = temp_x;
                y = temp_y;
            }
            path.push_back(cv::Point2f(start_x * grid_size + grid_size/2, start_y * grid_size + grid_size/2));
            
            // Reverse to get start to goal order
            std::reverse(path.begin(), path.end());
            return path;
        }
        
        // Explore neighbors
        for (const auto& neighbor : getNeighbors(current.x, current.y, grid_width, grid_height)) {
            int nx = neighbor.first;
            int ny = neighbor.second;
            
            // Skip if already visited
            std::string neighbor_key = std::to_string(nx) + "," + std::to_string(ny);
            if (closed_set.find(neighbor_key) != closed_set.end()) {
                continue;
            }
            
            // Calculate new g cost
            float new_g_cost = current.g_cost + grid_size;
            
            if (new_g_cost < g_cost[ny][nx]) {
                g_cost[ny][nx] = new_g_cost;
                parent[ny][nx] = {current.x, current.y};
                
                float h_cost = calculateHeuristic(nx, ny, goal_x, goal_y);
                PathNode neighbor_node(nx, ny, new_g_cost, h_cost, {current.x, current.y});
                open_set.push(neighbor_node);
            }
        }
    }
    
    // No path found
    return {};
}

std::vector<cv::Point2f> World::findPathFloodFill(const cv::Point2f& start, const cv::Point2f& goal, float grid_size) {
    // Convert world coordinates to grid coordinates
    int grid_width = static_cast<int>(width_ / grid_size);
    int grid_height = static_cast<int>(height_ / grid_size);
    
    int start_x = static_cast<int>(start.x / grid_size);
    int start_y = static_cast<int>(start.y / grid_size);
    int goal_x = static_cast<int>(goal.x / grid_size);
    int goal_y = static_cast<int>(goal.y / grid_size);
    
    // Validate grid positions
    if (!isValidGridPosition(start_x, start_y, grid_width, grid_height) ||
        !isValidGridPosition(goal_x, goal_y, grid_width, grid_height)) {
        return {};
    }
    
    // Initialize distance grid
    std::vector<std::vector<int>> distance(grid_height, std::vector<int>(grid_width, -1));
    std::vector<std::vector<std::pair<int, int>>> parent(grid_height, 
        std::vector<std::pair<int, int>>(grid_width, {-1, -1}));
    
    // BFS queue
    std::queue<std::pair<int, int>> queue;
    queue.push({start_x, start_y});
    distance[start_y][start_x] = 0;
    
    while (!queue.empty()) {
        auto [x, y] = queue.front();
        queue.pop();
        
        // Check if we reached the goal
        if (x == goal_x && y == goal_y) {
            // Reconstruct path
            std::vector<cv::Point2f> path;
            while (x != start_x || y != start_y) {
                path.push_back(cv::Point2f(x * grid_size + grid_size/2, y * grid_size + grid_size/2));
                int temp_x = parent[y][x].first;
                int temp_y = parent[y][x].second;
                x = temp_x;
                y = temp_y;
            }
            path.push_back(cv::Point2f(start_x * grid_size + grid_size/2, start_y * grid_size + grid_size/2));
            
            // Reverse to get start to goal order
            std::reverse(path.begin(), path.end());
            return path;
        }
        
        // Explore neighbors
        for (const auto& neighbor : getNeighbors(x, y, grid_width, grid_height)) {
            int nx = neighbor.first;
            int ny = neighbor.second;
            
            if (distance[ny][nx] == -1) {
                distance[ny][nx] = distance[y][x] + 1;
                parent[ny][nx] = {x, y};
                queue.push({nx, ny});
            }
        }
    }
    
    // No path found
    return {};
}

std::vector<cv::Point3f> World::findPathAStar3D(const cv::Point3f& start, const cv::Point3f& goal, float grid_size) {
    // Convert world coordinates to grid coordinates
    int grid_width = static_cast<int>(width_ / grid_size);
    int grid_height = static_cast<int>(height_ / grid_size);
    int grid_depth = static_cast<int>(depth_ / grid_size);
    
    int start_x = static_cast<int>(start.x / grid_size);
    int start_y = static_cast<int>(start.y / grid_size);
    int start_z = static_cast<int>(start.z / grid_size);
    int goal_x = static_cast<int>(goal.x / grid_size);
    int goal_y = static_cast<int>(goal.y / grid_size);
    int goal_z = static_cast<int>(goal.z / grid_size);
    
    // Validate grid positions
    if (!isValidGridPosition3D(start_x, start_y, start_z, grid_width, grid_height, grid_depth) ||
        !isValidGridPosition3D(goal_x, goal_y, goal_z, grid_width, grid_height, grid_depth)) {
        return {};
    }
    
    // Priority queue for A* algorithm
    std::priority_queue<PathNode3D, std::vector<PathNode3D>, std::greater<PathNode3D>> open_set;
    std::unordered_set<std::string> closed_set;
    
    // Initialize start node
    PathNode3D start_node(start_x, start_y, start_z, 0.0f, 
                         calculateHeuristic3D(start_x, start_y, start_z, goal_x, goal_y, goal_z));
    open_set.push(start_node);
    
    // Parent tracking for path reconstruction
    std::vector<std::vector<std::vector<std::tuple<int, int, int>>>> parent(
        grid_depth, std::vector<std::vector<std::tuple<int, int, int>>>(
            grid_height, std::vector<std::tuple<int, int, int>>(grid_width, {-1, -1, -1})));
    
    // Cost tracking
    std::vector<std::vector<std::vector<float>>> g_cost(
        grid_depth, std::vector<std::vector<float>>(
            grid_height, std::vector<float>(grid_width, std::numeric_limits<float>::max())));
    g_cost[start_z][start_y][start_x] = 0.0f;
    
    while (!open_set.empty()) {
        PathNode3D current = open_set.top();
        open_set.pop();
        
        // Create unique key for closed set
        std::string key = std::to_string(current.x) + "," + std::to_string(current.y) + "," + std::to_string(current.z);
        if (closed_set.find(key) != closed_set.end()) {
            continue;
        }
        closed_set.insert(key);
        
        // Check if we reached the goal
        if (current.x == goal_x && current.y == goal_y && current.z == goal_z) {
            // Reconstruct path
            std::vector<cv::Point3f> path;
            int x = goal_x, y = goal_y, z = goal_z;
            
            while (x != start_x || y != start_y || z != start_z) {
                path.push_back(cv::Point3f(x * grid_size + grid_size/2, 
                                         y * grid_size + grid_size/2, 
                                         z * grid_size + grid_size/2));
                auto [temp_x, temp_y, temp_z] = parent[z][y][x];
                x = temp_x;
                y = temp_y;
                z = temp_z;
            }
            path.push_back(cv::Point3f(start_x * grid_size + grid_size/2, 
                                     start_y * grid_size + grid_size/2, 
                                     start_z * grid_size + grid_size/2));
            
            // Reverse to get start to goal order
            std::reverse(path.begin(), path.end());
            return path;
        }
        
        // Explore neighbors
        for (const auto& neighbor : getNeighbors3D(current.x, current.y, current.z, 
                                                  grid_width, grid_height, grid_depth)) {
            auto [nx, ny, nz] = neighbor;
            
            // Skip if already visited
            std::string neighbor_key = std::to_string(nx) + "," + std::to_string(ny) + "," + std::to_string(nz);
            if (closed_set.find(neighbor_key) != closed_set.end()) {
                continue;
            }
            
            // Calculate new g cost (diagonal movement costs more)
            float movement_cost = (nx != current.x && ny != current.y && nz != current.z) ? 
                                 grid_size * 1.732f : grid_size; // sqrt(3) for diagonal
            float new_g_cost = current.g_cost + movement_cost;
            
            if (new_g_cost < g_cost[nz][ny][nx]) {
                g_cost[nz][ny][nx] = new_g_cost;
                parent[nz][ny][nx] = {current.x, current.y, current.z};
                
                float h_cost = calculateHeuristic3D(nx, ny, nz, goal_x, goal_y, goal_z);
                PathNode3D neighbor_node(nx, ny, nz, new_g_cost, h_cost, {current.x, current.y, current.z});
                open_set.push(neighbor_node);
            }
        }
    }
    
    // No path found
    return {};
}

std::vector<cv::Point3f> World::findPathFloodFill3D(const cv::Point3f& start, const cv::Point3f& goal, float grid_size) {
    // Convert world coordinates to grid coordinates
    int grid_width = static_cast<int>(width_ / grid_size);
    int grid_height = static_cast<int>(height_ / grid_size);
    int grid_depth = static_cast<int>(depth_ / grid_size);
    
    int start_x = static_cast<int>(start.x / grid_size);
    int start_y = static_cast<int>(start.y / grid_size);
    int start_z = static_cast<int>(start.z / grid_size);
    int goal_x = static_cast<int>(goal.x / grid_size);
    int goal_y = static_cast<int>(goal.y / grid_size);
    int goal_z = static_cast<int>(goal.z / grid_size);
    
    // Validate grid positions
    if (!isValidGridPosition3D(start_x, start_y, start_z, grid_width, grid_height, grid_depth) ||
        !isValidGridPosition3D(goal_x, goal_y, goal_z, grid_width, grid_height, grid_depth)) {
        return {};
    }
    
    // Initialize distance grid
    std::vector<std::vector<std::vector<int>>> distance(
        grid_depth, std::vector<std::vector<int>>(
            grid_height, std::vector<int>(grid_width, -1)));
    std::vector<std::vector<std::vector<std::tuple<int, int, int>>>> parent(
        grid_depth, std::vector<std::vector<std::tuple<int, int, int>>>(
            grid_height, std::vector<std::tuple<int, int, int>>(grid_width, {-1, -1, -1})));
    
    // BFS queue
    std::queue<std::tuple<int, int, int>> queue;
    queue.push({start_x, start_y, start_z});
    distance[start_z][start_y][start_x] = 0;
    
    while (!queue.empty()) {
        auto [x, y, z] = queue.front();
        queue.pop();
        
        // Check if we reached the goal
        if (x == goal_x && y == goal_y && z == goal_z) {
            // Reconstruct path
            std::vector<cv::Point3f> path;
            while (x != start_x || y != start_y || z != start_z) {
                path.push_back(cv::Point3f(x * grid_size + grid_size/2, 
                                         y * grid_size + grid_size/2, 
                                         z * grid_size + grid_size/2));
                auto [temp_x, temp_y, temp_z] = parent[z][y][x];
                x = temp_x;
                y = temp_y;
                z = temp_z;
            }
            path.push_back(cv::Point3f(start_x * grid_size + grid_size/2, 
                                     start_y * grid_size + grid_size/2, 
                                     start_z * grid_size + grid_size/2));
            
            // Reverse to get start to goal order
            std::reverse(path.begin(), path.end());
            return path;
        }
        
        // Explore neighbors
        for (const auto& neighbor : getNeighbors3D(x, y, z, grid_width, grid_height, grid_depth)) {
            auto [nx, ny, nz] = neighbor;
            
            if (distance[nz][ny][nx] == -1) {
                distance[nz][ny][nx] = distance[z][y][x] + 1;
                parent[nz][ny][nx] = {x, y, z};
                queue.push({nx, ny, nz});
            }
        }
    }
    
    // No path found
    return {};
}

bool World::isValidGridPosition(int x, int y, int grid_width, int grid_height) const {
    if (x < 0 || x >= grid_width || y < 0 || y >= grid_height) {
        return false;
    }
    
    // Check if position is occupied by obstacles
    cv::Point2f world_pos(x * 10.0f + 5.0f, y * 10.0f + 5.0f); // Assuming 10.0f grid size
    return !checkCollision(world_pos, 5.0f);
}

bool World::isValidGridPosition3D(int x, int y, int z, int grid_width, int grid_height, int grid_depth) const {
    if (x < 0 || x >= grid_width || y < 0 || y >= grid_height || z < 0 || z >= grid_depth) {
        return false;
    }
    
    // Check if position is occupied by obstacles
    cv::Point3f world_pos(x * 10.0f + 5.0f, y * 10.0f + 5.0f, z * 10.0f + 5.0f); // Assuming 10.0f grid size
    return !checkCollision(world_pos, 5.0f);
}

float World::calculateHeuristic(int x1, int y1, int x2, int y2) const {
    // Manhattan distance heuristic
    return std::abs(x2 - x1) + std::abs(y2 - y1);
}

float World::calculateHeuristic3D(int x1, int y1, int z1, int x2, int y2, int z2) const {
    // Manhattan distance heuristic for 3D
    return std::abs(x2 - x1) + std::abs(y2 - y1) + std::abs(z2 - z1);
}

std::vector<std::pair<int, int>> World::getNeighbors(int x, int y, int grid_width, int grid_height) const {
    std::vector<std::pair<int, int>> neighbors;
    
    // 8-directional movement
    const int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    
    for (int i = 0; i < 8; ++i) {
        int nx = x + dx[i];
        int ny = y + dy[i];
        
        if (isValidGridPosition(nx, ny, grid_width, grid_height)) {
            neighbors.push_back({nx, ny});
        }
    }
    
    return neighbors;
}

std::vector<std::tuple<int, int, int>> World::getNeighbors3D(int x, int y, int z, int grid_width, int grid_height, int grid_depth) const {
    std::vector<std::tuple<int, int, int>> neighbors;
    
    // 26-directional movement (including diagonals)
    const int dx[] = {-1, -1, -1, 0, 0, 0, 0, 1, 1, 1};
    const int dy[] = {-1, 0, 1, -1, 0, 1, -1, -1, 0, 1};
    const int dz[] = {-1, -1, 0, -1, 0, 1, 1, -1, 0, 0};
    
    for (int i = 0; i < 26; ++i) {
        int nx = x + dx[i];
        int ny = y + dy[i];
        int nz = z + dz[i];
        
        if (isValidGridPosition3D(nx, ny, nz, grid_width, grid_height, grid_depth)) {
            neighbors.push_back({nx, ny, nz});
        }
    }
    
    return neighbors;
}

void World::updateOccupancyGrid() {
    // Update occupancy grid based on current obstacles
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            cv::Point2f pos(x, y);
            occupancy_grid_2d_[y][x] = checkCollision(pos, 1.0f);
        }
    }
}

void World::update3DOccupancyGrid() {
    // Update 3D occupancy grid based on current obstacles
    for (int z = 0; z < depth_; ++z) {
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                cv::Point3f pos(x, y, z);
                occupancy_grid_3d_[z][y][x] = checkCollision(pos, 1.0f);
            }
        }
    }
}

void World::render3D(cv::Mat& output, const cv::Point3f& camera_pos, const cv::Point3f& camera_target) const {
    // Create a 3D visualization
    output = cv::Mat(height_, width_, CV_8UC3, cv::Scalar(200, 200, 255)); // Sky blue background
    
    // Simple 3D projection: project 3D obstacles to 2D screen
    for (const auto& obs : obstacles_) {
        // Calculate depth-based scaling - ensure camera is looking in the right direction
        float depth = camera_pos.z - obs.position.z; // Changed: camera.z - obstacle.z
        if (depth <= 0) continue; // Behind camera
        
        float scale = 2000.0f / (depth + 50.0f); // Dramatic perspective scaling for first-person drone view
        float screen_x = (obs.position.x - camera_pos.x) * scale + width_ / 2;
        float screen_y = (obs.position.y - camera_pos.y) * scale + height_ / 2;
        float screen_radius = obs.radius * scale;
        
        if (screen_x >= 0 && screen_x < width_ && screen_y >= 0 && screen_y < height_) {
            cv::Point2f screen_pos(screen_x, screen_y);
            
            // Color based on height
            cv::Scalar color;
            if (obs.is_vertical) {
                color = cv::Scalar(80, 80, 80); // Darker gray for buildings
            } else {
                color = cv::Scalar(0, 0, 0); // Black for other obstacles
            }
            
            // Draw buildings as outlined rectangles instead of solid circles
            if (obs.is_vertical) {
                float height_screen = obs.height * scale;
                cv::Point2f top_left(screen_x - screen_radius, screen_y - height_screen);
                cv::Point2f bottom_right(screen_x + screen_radius, screen_y);
                
                // Draw building outline
                cv::rectangle(output, top_left, bottom_right, color, 2);
                
                // Draw some windows (simple dots)
                for (int w = 0; w < 3; ++w) {
                    for (int h = 0; h < 5; ++h) {
                        float window_x = screen_x - screen_radius/2 + w * screen_radius/2;
                        float window_y = screen_y - height_screen/2 + h * height_screen/5;
                        cv::circle(output, cv::Point2f(window_x, window_y), 1, cv::Scalar(255, 255, 200), -1);
                    }
                }
            } else {
                // Draw other obstacles as small circles
                cv::circle(output, screen_pos, screen_radius, color, 1);
            }
        }
    }
    
    // Draw start and goal positions
    float start_scale = 2000.0f / (camera_pos.z - start_position_.z + 50.0f); // Dramatic scaling to match obstacles
    float goal_scale = 2000.0f / (camera_pos.z - goal_position_.z + 50.0f); // Dramatic scaling to match obstacles
    
    cv::Point2f start_screen((start_position_.x - camera_pos.x) * start_scale + width_ / 2,
                             (start_position_.y - camera_pos.y) * start_scale + height_ / 2);
    cv::Point2f goal_screen((goal_position_.x - camera_pos.x) * goal_scale + width_ / 2,
                            (goal_position_.y - camera_pos.y) * goal_scale + height_ / 2);
    
    cv::circle(output, start_screen, 10, cv::Scalar(0, 255, 0), -1);
    cv::circle(output, goal_screen, 10, cv::Scalar(0, 0, 255), -1);
}

void World::randomize3DEnvironment() {
    // Randomize 3D environment parameters
    std::uniform_real_distribution<float> height_dist(0.5f, 1.5f);
    std::uniform_real_distribution<float> variation_dist(0.5f, 1.5f);
    
    max_building_height_ *= height_dist(rng_);
    terrain_variation_ *= variation_dist(rng_);
    
    // Regenerate obstacles with new parameters
    if (!obstacles_.empty()) {
        auto current_type = MapType::SKYSCRAPER; // Default to skyscraper for 3D
        generateMap(current_type);
    }
}

} // namespace sim
