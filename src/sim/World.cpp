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
    // Create a more navigable maze with corridors
    obstacles_.clear();
    
    // Add some walls to create corridors
    for (int i = 0; i < 10; ++i) {
        // Horizontal walls
        float y = 100.0f + i * 50.0f;
        for (int j = 0; j < 8; ++j) {
            float x = 150.0f + j * 80.0f;
            Obstacle obs;
            obs.position = cv::Point2f(x, y);
            obs.radius = 8.0f;
            obs.is_moving = false;
            obs.velocity = cv::Point2f(0.0f, 0.0f);
            obs.max_speed = 0.0f;
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
        obs.position = cv::Point2f(x, y);
        obs.radius = 12.0f;
        obs.is_moving = false;
        obs.velocity = cv::Point2f(0.0f, 0.0f);
        obs.max_speed = 0.0f;
        obstacles_.push_back(obs);
        
        // Right wall
        x = 600.0f;
        obs.position = cv::Point2f(x, y);
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
    PathNode start_node(start_x, start_y, 0.0f, calculateHeuristic(start_x, start_y, goal_x, goal_y));
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

bool World::isValidGridPosition(int x, int y, int grid_width, int grid_height) const {
    if (x < 0 || x >= grid_width || y < 0 || y >= grid_height) {
        return false;
    }
    
    // Check if position is occupied by obstacles
    cv::Point2f world_pos(x * 10.0f + 5.0f, y * 10.0f + 5.0f); // Assuming 10.0f grid size
    return !checkCollision(world_pos, 5.0f);
}

float World::calculateHeuristic(int x1, int y1, int x2, int y2) const {
    // Manhattan distance heuristic
    return std::abs(x2 - x1) + std::abs(y2 - y1);
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

void World::updateOccupancyGrid() {
    // Update occupancy grid based on current obstacles
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            cv::Point2f pos(x, y);
            occupancy_grid_[y][x] = checkCollision(pos, 1.0f);
        }
    }
}

} // namespace sim
