#include <iostream>
#include <memory>
#include "sim/World.h"
#include "sim/Drone.h"

int main() {
    std::cout << "=== Pathfinding Algorithm Test ===" << std::endl;
    
    // Create world
    auto world = std::make_shared<sim::World>(800, 600);
    world->generateMap(sim::MapType::MAZE);
    std::cout << "World created: " << world->getSize().width << "x" << world->getSize().height << std::endl;
    
    // Get start and goal positions
    cv::Point2f start_pos = world->getStartPosition();
    cv::Point2f goal_pos = world->getGoalPosition();
    std::cout << "Start position: (" << start_pos.x << ", " << start_pos.y << ")" << std::endl;
    std::cout << "Goal position: (" << goal_pos.x << ", " << goal_pos.y << ")" << std::endl;
    
    // Test A* pathfinding
    std::cout << "\n--- Testing A* Pathfinding ---" << std::endl;
    
    auto astar_path = world->findPathAStar(start_pos, goal_pos, 10.0f);
    std::cout << "A* path found: " << astar_path.size() << " waypoints" << std::endl;
    
    if (!astar_path.empty()) {
        std::cout << "First 5 waypoints:" << std::endl;
        for (int i = 0; i < std::min(5, (int)astar_path.size()); ++i) {
            cv::Point2f wp = astar_path[i];
            std::cout << "  " << i << ": (" << wp.x << ", " << wp.y << ") - In bounds: " 
                      << (world->isInBounds(wp) ? "YES" : "NO") << std::endl;
        }
        
        if (astar_path.size() > 5) {
            std::cout << "Last 5 waypoints:" << std::endl;
            for (int i = std::max(0, (int)astar_path.size() - 5); i < astar_path.size(); ++i) {
                cv::Point2f wp = astar_path[i];
                std::cout << "  " << i << ": (" << wp.x << ", " << wp.y << ") - In bounds: " 
                          << (world->isInBounds(wp) ? "YES" : "NO") << std::endl;
            }
        }
        
        // Check for any out-of-bounds waypoints
        bool all_in_bounds = true;
        for (const auto& wp : astar_path) {
            if (!world->isInBounds(wp)) {
                std::cout << "*** OUT OF BOUNDS WAYPOINT: (" << wp.x << ", " << wp.y << ") ***" << std::endl;
                all_in_bounds = false;
            }
        }
        std::cout << "All waypoints in bounds: " << (all_in_bounds ? "YES" : "NO") << std::endl;
    }
    
    // Test Flood Fill pathfinding
    std::cout << "\n--- Testing Flood Fill Pathfinding ---" << std::endl;
    
    auto floodfill_path = world->findPathFloodFill(start_pos, goal_pos, 10.0f);
    std::cout << "Flood Fill path found: " << floodfill_path.size() << " waypoints" << std::endl;
    
    if (!floodfill_path.empty()) {
        std::cout << "First 5 waypoints:" << std::endl;
        for (int i = 0; i < std::min(5, (int)floodfill_path.size()); ++i) {
            cv::Point2f wp = floodfill_path[i];
            std::cout << "  " << i << ": (" << wp.x << ", " << wp.y << ") - In bounds: " 
                      << (world->isInBounds(wp) ? "YES" : "NO") << std::endl;
        }
        
        if (floodfill_path.size() > 5) {
            std::cout << "Last 5 waypoints:" << std::endl;
            for (int i = std::max(0, (int)floodfill_path.size() - 5); i < floodfill_path.size(); ++i) {
                cv::Point2f wp = floodfill_path[i];
                std::cout << "  " << i << ": (" << wp.x << ", " << wp.y << ") - In bounds: " 
                          << (world->isInBounds(wp) ? "YES" : "NO") << std::endl;
            }
        }
        
        // Check for any out-of-bounds waypoints
        bool all_in_bounds = true;
        for (const auto& wp : floodfill_path) {
            if (!world->isInBounds(wp)) {
                std::cout << "*** OUT OF BOUNDS WAYPOINT: (" << wp.x << ", " << wp.y << ") ***" << std::endl;
                all_in_bounds = false;
            }
        }
        std::cout << "All waypoints in bounds: " << (all_in_bounds ? "YES" : "NO") << std::endl;
    }
    
    // Test grid conversion
    std::cout << "\n--- Testing Grid Conversion ---" << std::endl;
    
    // Test some world coordinates
    std::vector<cv::Point2f> test_points = {
        cv::Point2f(0, 0),      // Top-left corner
        cv::Point2f(400, 300),  // Center
        cv::Point2f(800, 600),  // Bottom-right corner
        cv::Point2f(-100, -100), // Out of bounds
        cv::Point2f(900, 700)   // Out of bounds
    };
    
    for (const auto& point : test_points) {
        bool in_bounds = world->isInBounds(point);
        std::cout << "Point (" << point.x << ", " << point.y << "): In bounds = " 
                  << (in_bounds ? "YES" : "NO") << std::endl;
    }
    
    std::cout << "\n=== Test Complete ===" << std::endl;
    return 0;
}
