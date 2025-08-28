#include <iostream>
#include <memory>
#include "bridge/Environment.h"
#include "agent/QLearningAgent.h"
#include "sim/World.h"
#include "sim/Drone.h"

int main() {
    std::cout << "=== Episode Loop Test ===" << std::endl;
    
    // Create world with maze (exactly like failing test)
    auto world = std::make_shared<sim::World>(800, 600);
    world->generateMap(sim::MapType::MAZE);
    std::cout << "World created: " << world->getSize().width << "x" << world->getSize().height << std::endl;
    
    // Create drone (exactly like failing test)
    auto drone = std::make_shared<sim::Drone>(world->getStartPosition(), 0.0f);
    drone->setWorld(world.get());
    
    // Create Q-Learning agent (exactly like failing test)
    agent::AgentConfig agent_config;
    agent_config.use_vision = false;
    agent_config.learning_rate = 0.1f;
    agent_config.discount_factor = 0.99f;
    agent_config.epsilon = 0.1f;
    agent_config.replay_buffer_size = 1000;
    
    auto agent = std::make_shared<agent::QLearningAgent>(agent_config);
    
    // Create environment (exactly like failing test)
    bridge::EnvironmentConfig env_config;
    env_config.max_steps_per_episode = 1000;
    env_config.time_step = 0.1f;
    env_config.render_episodes = true;
    env_config.goal_reward = 100.0f;
    env_config.collision_penalty = -50.0f;
    env_config.progress_reward = 1.0f;
    env_config.time_penalty = -0.1f;
    
    auto env = std::make_shared<bridge::Environment>(env_config);
    env->setWorld(world);
    env->setDrone(drone);
    env->setAgent(agent);
    
    // Enable pathfinding (exactly like failing test)
    env->setPathfindingAlgorithm("astar");
    env->setUsePathfinding(true);
    
    std::cout << "Environment created and configured (exactly like failing test)" << std::endl;
    
    // Test the exact episode conditions
    std::cout << "\n--- Testing Episode Conditions ---" << std::endl;
    
    // Run a few steps to see where it breaks
    for (int step = 0; step < 20; ++step) {
        cv::Point2f current_pos = drone->getState().position;
        float distance_to_goal = drone->getDistanceToGoal(world->getGoalPosition());
        
        std::cout << "Step " << step << ":" << std::endl;
        std::cout << "  Position: (" << current_pos.x << ", " << current_pos.y << ")" << std::endl;
        std::cout << "  Distance to goal: " << distance_to_goal << std::endl;
        std::cout << "  In bounds: " << (world->isInBounds(current_pos) ? "YES" : "NO") << std::endl;
        
        // Check if we're about to go out of bounds
        if (!world->isInBounds(current_pos)) {
            std::cout << "  *** DRONE ALREADY OUT OF BOUNDS! ***" << std::endl;
            break;
        }
        
        // Take step
        env->step(agent);
        
        cv::Point2f new_pos = drone->getState().position;
        std::cout << "  After step:" << std::endl;
        std::cout << "    Position: (" << new_pos.x << ", " << new_pos.y << ")" << std::endl;
        std::cout << "    In bounds: " << (world->isInBounds(new_pos) ? "YES" : "NO") << std::endl;
        
        if (!world->isInBounds(new_pos)) {
            std::cout << "  *** DRONE WENT OUT OF BOUNDS AFTER STEP! ***" << std::endl;
            std::cout << "  *** This is where the corruption happens! ***" << std::endl;
            break;
        }
        
        std::cout << std::endl;
    }
    
    // Test environment reset
    std::cout << "\n--- Testing Environment Reset ---" << std::endl;
    
    cv::Point2f before_reset_pos = drone->getState().position;
    std::cout << "Before reset position: (" << before_reset_pos.x << ", " << before_reset_pos.y << ")" << std::endl;
    std::cout << "Before reset in bounds: " << (world->isInBounds(before_reset_pos) ? "YES" : "NO") << std::endl;
    
    env->reset();
    
    cv::Point2f after_reset_pos = drone->getState().position;
    std::cout << "After reset position: (" << after_reset_pos.x << ", " << after_reset_pos.y << ")" << std::endl;
    std::cout << "After reset in bounds: " << (world->isInBounds(after_reset_pos) ? "YES" : "NO") << std::endl;
    
    // Test another step after reset
    std::cout << "\n--- Testing Step After Reset ---" << std::endl;
    
    env->step(agent);
    
    cv::Point2f final_pos = drone->getState().position;
    std::cout << "Final position: (" << final_pos.x << ", " << final_pos.y << ")" << std::endl;
    std::cout << "Final position in bounds: " << (world->isInBounds(final_pos) ? "YES" : "NO") << std::endl;
    
    std::cout << "\n=== Test Complete ===" << std::endl;
    return 0;
}
