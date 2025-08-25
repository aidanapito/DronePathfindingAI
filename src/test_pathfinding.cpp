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
    
    auto env = std::make_shared<bridge::Environment>(env_config);
    env->setWorld(world);
    env->setDrone(drone);
    env->setAgent(agent);
    
    // Test pathfinding
    std::cout << "\n--- Testing A* Pathfinding ---" << std::endl;
    env->setPathfindingAlgorithm("astar");
    env->setUsePathfinding(true);
    
    // Run a few episodes to test the integration
    for (int episode = 0; episode < 3; ++episode) {
        std::cout << "\nEpisode " << episode + 1 << std::endl;
        
        auto result = env->runEpisode(agent);
        
        std::cout << "  Steps: " << result.steps << std::endl;
        std::cout << "  Success: " << (result.success ? "Yes" : "No") << std::endl;
        std::cout << "  Total Reward: " << result.total_reward << std::endl;
        std::cout << "  Final Distance: " << result.final_distance << std::endl;
        
        // Show pathfinding info
        auto optimal_path = env->getOptimalPath();
        std::cout << "  Optimal Path Waypoints: " << optimal_path.size() << std::endl;
        
        // Show agent debug info
        std::cout << "  Agent Debug Info:" << std::endl;
        std::cout << agent->getDebugInfo() << std::endl;
    }
    
    // Test flood fill pathfinding
    std::cout << "\n--- Testing Flood Fill Pathfinding ---" << std::endl;
    env->setPathfindingAlgorithm("floodfill");
    
    auto result = env->runEpisode(agent);
    std::cout << "Flood Fill Episode Results:" << std::endl;
    std::cout << "  Steps: " << result.steps << std::endl;
    std::cout << "  Success: " << (result.success ? "Yes" : "No") << std::endl;
    std::cout << "  Total Reward: " << result.total_reward << std::endl;
    
    std::cout << "\n=== Test Complete ===" << std::endl;
    return 0;
}
