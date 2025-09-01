#include "AI/PathPlanner.h"
#include <iostream>
#include <cmath>
#include <algorithm>

PathPlanner::PathPlanner()
    : grid_resolution_(DEFAULT_GRID_RESOLUTION)
    , max_search_iterations_(DEFAULT_MAX_SEARCH_ITERATIONS)
    , safety_margin_(DEFAULT_SAFETY_MARGIN)
    , line_of_sight_step_size_(DEFAULT_LINE_OF_SIGHT_STEP_SIZE) {
}

std::vector<glm::vec3> PathPlanner::findPath(const glm::vec3& start, const glm::vec3& goal, const World& world) {
    // First try direct line of sight
    if (isLineOfSight(start, goal, world)) {
        std::vector<glm::vec3> direct_path = {start, goal};
        return optimizePath(direct_path, world);
    }
    
    // Use A* for complex paths
    std::vector<glm::vec3> raw_path = astarSearch(start, goal, world);
    if (raw_path.empty()) {
        return raw_path;
    }
    
    // Optimize the path
    return optimizePath(raw_path, world);
}

std::vector<glm::vec3> PathPlanner::optimizePath(const std::vector<glm::vec3>& path, const World& world) {
    if (path.size() < 3) {
        return path;
    }
    
    // First simplify the path by removing unnecessary waypoints
    std::vector<glm::vec3> simplified = simplifyPath(path, world);
    
    // Then smooth the path
    return smoothPath(simplified, world);
}

std::vector<glm::vec3> PathPlanner::smoothPath(const std::vector<glm::vec3>& path, const World& world) {
    if (path.size() < 3) {
        return path;
    }
    
    std::vector<glm::vec3> smoothed = path;
    
    // Simple smoothing: try to connect non-adjacent points if line of sight exists
    for (size_t i = 0; i < smoothed.size() - 2; ++i) {
        if (isLineOfSight(smoothed[i], smoothed[i + 2], world)) {
            smoothed.erase(smoothed.begin() + i + 1);
            --i; // Recheck this index
        }
    }
    
    return smoothed;
}

bool PathPlanner::isPathValid(const std::vector<glm::vec3>& path, const World& world) const {
    if (path.size() < 2) {
        return false;
    }
    
    for (size_t i = 0; i < path.size() - 1; ++i) {
        if (!isLineOfSight(path[i], path[i + 1], world)) {
            return false;
        }
    }
    
    return true;
}

float PathPlanner::calculatePathLength(const std::vector<glm::vec3>& path) const {
    float length = 0.0f;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        length += glm::length(path[i + 1] - path[i]);
    }
    return length;
}

std::vector<glm::vec3> PathPlanner::astarSearch(const glm::vec3& start, const glm::vec3& goal, const World& world) {
    // Priority queue for open set (nodes to explore)
    std::priority_queue<std::shared_ptr<PathNode>, 
                       std::vector<std::shared_ptr<PathNode>>, 
                       PathNodeCompare> open_set;
    
    // Hash set for closed set (explored nodes)
    std::unordered_set<std::shared_ptr<PathNode>, PathNodeHash, PathNodeEqual> closed_set;
    
    // Hash set for open set tracking
    std::unordered_set<std::shared_ptr<PathNode>, PathNodeHash, PathNodeEqual> open_set_tracker;
    
    // Create start node
    auto start_node = std::make_shared<PathNode>(start.x, start.y, start.z);
    start_node->h_cost = calculateHeuristic(start, goal);
    start_node->f_cost = start_node->h_cost;
    
    open_set.push(start_node);
    open_set_tracker.insert(start_node);
    
    int iterations = 0;
    
    while (!open_set.empty() && iterations < max_search_iterations_) {
        auto current = open_set.top();
        open_set.pop();
        open_set_tracker.erase(current);
        
        // Check if we reached the goal
        if (glm::length(glm::vec3(current->x, current->y, current->z) - goal) < grid_resolution_) {
            return reconstructPath(current);
        }
        
        closed_set.insert(current);
        
        // Get neighbors
        auto neighbors = getNeighbors(current, world);
        
        for (auto& neighbor : neighbors) {
            // Skip if already explored
            if (closed_set.find(neighbor) != closed_set.end()) {
                continue;
            }
            
            float tentative_g_cost = current->g_cost + calculateCost(
                glm::vec3(current->x, current->y, current->z),
                glm::vec3(neighbor->x, neighbor->y, neighbor->z),
                world
            );
            
            // Check if this path is better than previous ones
            bool is_better_path = false;
            
            if (open_set_tracker.find(neighbor) == open_set_tracker.end()) {
                // New node
                open_set_tracker.insert(neighbor);
                is_better_path = true;
            } else if (tentative_g_cost < neighbor->g_cost) {
                // Better path found
                is_better_path = true;
            }
            
            if (is_better_path) {
                neighbor->parent = current;
                neighbor->g_cost = tentative_g_cost;
                neighbor->h_cost = calculateHeuristic(glm::vec3(neighbor->x, neighbor->y, neighbor->z), goal);
                neighbor->f_cost = neighbor->g_cost + neighbor->h_cost;
                
                open_set.push(neighbor);
            }
        }
        
        ++iterations;
    }
    
    std::cout << "A* search failed after " << iterations << " iterations" << std::endl;
    return std::vector<glm::vec3>();
}

std::vector<std::shared_ptr<PathNode>> PathPlanner::getNeighbors(const std::shared_ptr<PathNode>& node, const World& world) {
    std::vector<std::shared_ptr<PathNode>> neighbors;
    
    // Generate neighbors in a 3D grid pattern
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) {
                    continue; // Skip the node itself
                }
                
                float new_x = node->x + dx * grid_resolution_;
                float new_y = node->y + dy * grid_resolution_;
                float new_z = node->z + dz * grid_resolution_;
                
                glm::vec3 new_pos(new_x, new_y, new_z);
                
                if (isPositionValid(new_pos, world)) {
                    neighbors.push_back(std::make_shared<PathNode>(new_x, new_y, new_z));
                }
            }
        }
    }
    
    return neighbors;
}

float PathPlanner::calculateHeuristic(const glm::vec3& from, const glm::vec3& to) const {
    // Use Euclidean distance as heuristic
    return glm::length(to - from);
}

float PathPlanner::calculateCost(const glm::vec3& from, const glm::vec3& to, const World& world) const {
    float distance = glm::length(to - from);
    
    // Add penalty for being close to obstacles
    float obstacle_penalty = 0.0f;
    if (world.checkCollision(to.x, to.y, to.z, safety_margin_)) {
        obstacle_penalty = 1000.0f; // High penalty for collision
    } else if (world.checkCollision(to.x, to.y, to.z, safety_margin_ * 2)) {
        obstacle_penalty = 100.0f; // Medium penalty for being close
    }
    
    return distance + obstacle_penalty;
}

std::vector<glm::vec3> PathPlanner::reconstructPath(const std::shared_ptr<PathNode>& goal_node) const {
    std::vector<glm::vec3> path;
    auto current = goal_node;
    
    while (current != nullptr) {
        path.push_back(glm::vec3(current->x, current->y, current->z));
        current = current->parent;
    }
    
    // Reverse to get start to goal order
    std::reverse(path.begin(), path.end());
    
    return path;
}

glm::vec3 PathPlanner::snapToGrid(const glm::vec3& position) const {
    return glm::vec3(
        round(position.x / grid_resolution_) * grid_resolution_,
        round(position.y / grid_resolution_) * grid_resolution_,
        round(position.z / grid_resolution_) * grid_resolution_
    );
}

bool PathPlanner::isPositionValid(const glm::vec3& position, const World& world) const {
    // Check if position is within world bounds
    if (position.x < 0 || position.x > world.getWidth() ||
        position.y < 0 || position.y > world.getDepth() ||
        position.z < 0 || position.z > world.getHeight()) {
        return false;
    }
    
    // Check collision with safety margin
    return !world.checkCollision(position.x, position.y, position.z, safety_margin_);
}

bool PathPlanner::isLineOfSight(const glm::vec3& from, const glm::vec3& to, const World& world) const {
    glm::vec3 direction = to - from;
    float distance = glm::length(direction);
    
    if (distance < line_of_sight_step_size_) {
        return isPositionValid(to, world);
    }
    
    direction = glm::normalize(direction);
    
    // Step along the line checking for obstacles
    for (float step = line_of_sight_step_size_; step < distance; step += line_of_sight_step_size_) {
        glm::vec3 check_point = from + direction * step;
        
        if (!isPositionValid(check_point, world)) {
            return false;
        }
    }
    
    return isPositionValid(to, world);
}

std::vector<glm::vec3> PathPlanner::simplifyPath(const std::vector<glm::vec3>& path, const World& world) {
    if (path.size() < 3) {
        return path;
    }
    
    std::vector<glm::vec3> simplified;
    simplified.push_back(path.front());
    
    for (size_t i = 1; i < path.size() - 1; ++i) {
        // Check if we can skip this point
        if (!isLineOfSight(simplified.back(), path[i + 1], world)) {
            simplified.push_back(path[i]);
        }
    }
    
    simplified.push_back(path.back());
    
    return simplified;
}

std::vector<glm::vec3> PathPlanner::addIntermediatePoints(const std::vector<glm::vec3>& path, const World& world) {
    std::vector<glm::vec3> result;
    
    for (size_t i = 0; i < path.size() - 1; ++i) {
        result.push_back(path[i]);
        
        // Add intermediate points if segments are too long
        glm::vec3 segment = path[i + 1] - path[i];
        float segment_length = glm::length(segment);
        
        if (segment_length > MAX_PATH_SEGMENT_LENGTH) {
            int num_intermediate = static_cast<int>(segment_length / MAX_PATH_SEGMENT_LENGTH);
            glm::vec3 step = segment / static_cast<float>(num_intermediate + 1);
            
            for (int j = 1; j <= num_intermediate; ++j) {
                glm::vec3 intermediate = path[i] + step * static_cast<float>(j);
                result.push_back(intermediate);
            }
        }
    }
    
    result.push_back(path.back());
    return result;
}
