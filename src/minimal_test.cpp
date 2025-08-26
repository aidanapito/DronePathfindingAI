#include <iostream>
#include <memory>
#include "sim/World.h"
#include "sim/Drone.h"

int main() {
    std::cout << "=== Minimal Drone Test ===" << std::endl;
    
    // Create a simple world
    auto world = std::make_shared<sim::World>(800, 600);
    std::cout << "World created: " << world->getSize().width << "x" << world->getSize().height << std::endl;
    
    // Get start and goal positions
    cv::Point2f start_pos = world->getStartPosition();
    cv::Point2f goal_pos = world->getGoalPosition();
    std::cout << "Start position: (" << start_pos.x << ", " << start_pos.y << ")" << std::endl;
    std::cout << "Goal position: (" << goal_pos.x << ", " << goal_pos.y << ")" << std::endl;
    
    // Check if positions are within bounds
    std::cout << "Start in bounds: " << (world->isInBounds(start_pos) ? "YES" : "NO") << std::endl;
    std::cout << "Goal in bounds: " << (world->isInBounds(goal_pos) ? "YES" : "NO") << std::endl;
    
    // Create drone at start position
    auto drone = std::make_shared<sim::Drone>(start_pos, 0.0f);
    drone->setWorld(world.get());
    
    // Test basic drone movement
    std::cout << "\n--- Testing Basic Movement ---" << std::endl;
    
    for (int step = 0; step < 10; ++step) {
        cv::Point2f current_pos = drone->getState().position;
        float current_heading = drone->getState().heading;
        float distance_to_goal = drone->getDistanceToGoal(goal_pos);
        
        std::cout << "Step " << step << ":" << std::endl;
        std::cout << "  Position: (" << current_pos.x << ", " << current_pos.y << ")" << std::endl;
        std::cout << "  Heading: " << current_heading << " radians (" << (current_heading * 180.0f / M_PI) << " degrees)" << std::endl;
        std::cout << "  Distance to goal: " << distance_to_goal << std::endl;
        std::cout << "  In bounds: " << (world->isInBounds(current_pos) ? "YES" : "NO") << std::endl;
        
        // Move forward
        drone->update(0.1f, 1.0f, 0.0f);
        
        std::cout << "  After move:" << std::endl;
        current_pos = drone->getState().position;
        std::cout << "    Position: (" << current_pos.x << ", " << current_pos.y << ")" << std::endl;
        std::cout << "    In bounds: " << (world->isInBounds(current_pos) ? "YES" : "NO") << std::endl;
        std::cout << std::endl;
    }
    
    // Test turning
    std::cout << "\n--- Testing Turning ---" << std::endl;
    
    for (int step = 0; step < 5; ++step) {
        cv::Point2f current_pos = drone->getState().position;
        float current_heading = drone->getState().heading;
        
        std::cout << "Step " << step << ":" << std::endl;
        std::cout << "  Position: (" << current_pos.x << ", " << current_pos.y << ")" << std::endl;
        std::cout << "  Heading: " << current_heading << " radians (" << (current_heading * 180.0f / M_PI) << " degrees)" << std::endl;
        
        // Turn left
        drone->update(0.1f, 0.0f, 1.0f);
        
        std::cout << "  After turn:" << std::endl;
        current_pos = drone->getState().position;
        current_heading = drone->getState().heading;
        std::cout << "    Position: (" << current_pos.x << ", " << current_pos.y << ")" << std::endl;
        std::cout << "    Heading: " << current_heading << " radians (" << (current_heading * 180.0f / M_PI) << " degrees)" << std::endl;
        std::cout << "    In bounds: " << (world->isInBounds(current_pos) ? "YES" : "NO") << std::endl;
        std::cout << std::endl;
    }
    
    // Test boundary conditions
    std::cout << "\n--- Testing Boundary Conditions ---" << std::endl;
    
    // Try to move outside bounds
    std::cout << "Testing wouldCollide for out-of-bounds movement:" << std::endl;
    bool would_collide = drone->wouldCollide(1.0f, 0.0f, 1.0f);
    std::cout << "  Would collide (forward): " << (would_collide ? "YES" : "NO") << std::endl;
    
    // Test emergency stop
    std::cout << "Testing emergency stop:" << std::endl;
    bool emergency_stop = drone->isEmergencyStop();
    std::cout << "  Emergency stop: " << (emergency_stop ? "YES" : "NO") << std::endl;
    
    std::cout << "\n=== Test Complete ===" << std::endl;
    return 0;
}
