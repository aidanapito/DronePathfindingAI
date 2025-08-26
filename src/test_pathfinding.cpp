#include "bridge/Environment.h"
#include "agent/QLearningAgent.h"
#include "sim/World.h"
#include "sim/Drone.h"
#include <iostream>
#include <memory>

int main() {
    std::cout << "=== Drone Pathfinding AI Test ===" << std::endl;
    
    // Create world with maze
    auto world = std::make_shared<sim::World>(800, 600);
    world->generateMap(sim::MapType::MAZE);
    
    // Create drone
    auto drone = std::make_shared<sim::Drone>(world->getStartPosition(), 0.0f);
    
    // Create Q-Learning agent
    agent::AgentConfig agent_config;
    agent_config.use_vision = false;  // Track A: occupancy grid
    agent_config.learning_rate = 0.1f;
    agent_config.discount_factor = 0.99f;
    agent_config.epsilon = 0.1f;
    agent_config.replay_buffer_size = 1000;
    
    auto agent = std::make_shared<agent::QLearningAgent>(agent_config);
    
    // Create environment
    bridge::EnvironmentConfig env_config;
    env_config.max_steps_per_episode = 1000;
    env_config.time_step = 0.1f;
    env_config.render_episodes = true;
    env_config.goal_reward = 100.0f;
    env_config.collision_penalty = -50.0f;
    env_config.progress_reward = 1.0f;
    env_config.time_penalty = -0.1f;
    
    // Test different action execution configurations
    std::cout << "\n--- Testing Action Execution Configurations ---" << std::endl;
    
    // CRITICAL FIX: Use much more conservative settings to prevent out-of-bounds movement
    // The default values (throttle_scale=1.0, yaw_rate_scale=1.0) are too aggressive
    // and cause the drone to move 100 units per step, overwhelming boundary checks
    env_config.throttle_scale = 0.1f;        // 10% throttle = 10 units per step
    env_config.yaw_rate_scale = 0.1f;        // 10% yaw rate = 0.1 radians per step
    env_config.enable_safety_checks = true;
    env_config.action_log_frequency = 5;
    
    std::cout << "Testing with VERY conservative settings:" << std::endl;
    std::cout << "  Throttle Scale: " << env_config.throttle_scale << " (10 units/step)" << std::endl;
    std::cout << "  Yaw Rate Scale: " << env_config.yaw_rate_scale << " (0.1 rad/step)" << std::endl;
    std::cout << "  Safety Checks: " << (env_config.enable_safety_checks ? "Enabled" : "Disabled") << std::endl;
    std::cout << "  Action Log Frequency: " << env_config.action_log_frequency << std::endl;
    
    // CRITICAL FIX: Create a new environment with the updated config
    // The old environment was created with default values (throttle_scale=1.0, yaw_rate_scale=1.0)
    // which caused the drone to move too fast and go out of bounds
    std::cout << "\n--- Creating New Environment with Conservative Settings ---" << std::endl;
    auto env_conservative = std::make_shared<bridge::Environment>(env_config);
    env_conservative->setWorld(world);
    env_conservative->setDrone(drone);
    env_conservative->setAgent(agent);
    env_conservative->setPathfindingAlgorithm("astar");
    env_conservative->setUsePathfinding(true);
    
    std::cout << "New environment created with conservative settings" << std::endl;
    
    // Test pathfinding
    std::cout << "\n--- Testing A* Pathfinding ---" << std::endl;
    
    // Run a few episodes to test the integration
    for (int episode = 0; episode < 3; ++episode) {
        std::cout << "\nEpisode " << episode + 1 << std::endl;
        
        auto result = env_conservative->runEpisode(agent);
        
        std::cout << "  Steps: " << result.steps << std::endl;
        std::cout << "  Success: " << (result.success ? "Yes" : "No") << std::endl;
        std::cout << "  Total Reward: " << result.total_reward << std::endl;
        std::cout << "  Final Distance: " << result.final_distance << std::endl;
        
        // Show pathfinding info
        auto optimal_path = env_conservative->getOptimalPath();
        std::cout << "  Optimal Path Waypoints: " << optimal_path.size() << std::endl;
        
        // Show action execution statistics
        env_conservative->printActionStats();
        
        // Show agent debug info
        std::cout << "  Agent Debug Info:" << std::endl;
        std::cout << agent->getDebugInfo() << std::endl;
    }
    
    // Test flood fill pathfinding
    std::cout << "\n--- Testing Flood Fill Pathfinding ---" << std::endl;
    env_conservative->setPathfindingAlgorithm("floodfill");
    
    auto result = env_conservative->runEpisode(agent);
    std::cout << "Flood Fill Episode Results:" << std::endl;
    std::cout << "  Steps: " << result.steps << std::endl;
    std::cout << "  Success: " << (result.success ? "Yes" : "No") << std::endl;
    std::cout << "  Total Reward: " << result.total_reward << std::endl;
    
    // Show action execution statistics for flood fill
    env_conservative->printActionStats();
    
    std::cout << "\n=== Test Complete ===" << std::endl;
    return 0;
}
