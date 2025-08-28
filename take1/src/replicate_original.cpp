#include <iostream>
#include <memory>
#include "bridge/Environment.h"
#include "agent/QLearningAgent.h"
#include "sim/World.h"
#include "sim/Drone.h"

void checkDronePosition(const std::string& step_name, const std::shared_ptr<sim::Drone>& drone, const std::shared_ptr<sim::World>& world) {
    cv::Point2f pos = drone->getState().position;
    bool in_bounds = world->isInBounds(pos);
    std::cout << step_name << ": Position (" << pos.x << ", " << pos.y << ") - In bounds: " << (in_bounds ? "YES" : "NO") << std::endl;
    
    if (!in_bounds) {
        std::cout << "  *** DRONE CORRUPTED AT: " << step_name << " ***" << std::endl;
    }
}

int main() {
    std::cout << "=== Replicate Original Test Flow ===" << std::endl;
    
    // EXACTLY replicate the original test flow
    
    // Step 1: Create world with maze
    std::cout << "\n--- Step 1: Create World with Maze ---" << std::endl;
    auto world = std::make_shared<sim::World>(800, 600);
    world->generateMap(sim::MapType::MAZE);
    std::cout << "World created: " << world->getSize().width << "x" << world->getSize().height << std::endl;
    
    cv::Point2f start_pos = world->getStartPosition();
    cv::Point2f goal_pos = world->getGoalPosition();
    std::cout << "Start position: (" << start_pos.x << ", " << start_pos.y << ")" << std::endl;
    std::cout << "Goal position: (" << goal_pos.x << ", " << goal_pos.y << ")" << std::endl;
    
    // Step 2: Create drone
    std::cout << "\n--- Step 2: Create Drone ---" << std::endl;
    auto drone = std::make_shared<sim::Drone>(start_pos, 0.0f);
    checkDronePosition("After drone creation", drone, world);
    
    // Step 3: Create Q-Learning agent
    std::cout << "\n--- Step 3: Create Q-Learning Agent ---" << std::endl;
    agent::AgentConfig agent_config;
    agent_config.use_vision = false;  // Track A: occupancy grid
    agent_config.learning_rate = 0.1f;
    agent_config.discount_factor = 0.99f;
    agent_config.epsilon = 0.1f;
    agent_config.replay_buffer_size = 1000;
    
    auto agent = std::make_shared<agent::QLearningAgent>(agent_config);
    checkDronePosition("After agent creation", drone, world);
    
    // Step 4: Create environment
    std::cout << "\n--- Step 4: Create Environment ---" << std::endl;
    bridge::EnvironmentConfig env_config;
    env_config.max_steps_per_episode = 1000;
    env_config.time_step = 0.1f;
    env_config.render_episodes = true;
    env_config.goal_reward = 100.0f;
    env_config.collision_penalty = -50.0f;
    env_config.progress_reward = 1.0f;
    env_config.time_penalty = -0.1f;
    
    auto env = std::make_shared<bridge::Environment>(env_config);
    checkDronePosition("After environment creation", drone, world);
    
    // Step 5: Set world, drone, and agent on environment
    std::cout << "\n--- Step 5: Set World, Drone, and Agent on Environment ---" << std::endl;
    env->setWorld(world);
    env->setDrone(drone);
    env->setAgent(agent);
    checkDronePosition("After setting world/drone/agent", drone, world);
    
    // Step 6: Test different action execution configurations (EXACTLY like original)
    std::cout << "\n--- Step 6: Test Different Action Execution Configurations ---" << std::endl;
    
    // Test with conservative settings (EXACTLY like original)
    env_config.throttle_scale = 0.5f;
    env_config.yaw_rate_scale = 0.5f;
    env_config.enable_safety_checks = true;
    env_config.action_log_frequency = 5;
    
    std::cout << "Testing with conservative settings:" << std::endl;
    std::cout << "  Throttle Scale: " << env_config.throttle_scale << std::endl;
    std::cout << "  Yaw Rate Scale: " << env_config.yaw_rate_scale << std::endl;
    std::cout << "  Safety Checks: " << (env_config.enable_safety_checks ? "Enabled" : "Disabled") << std::endl;
    std::cout << "  Action Log Frequency: " << env_config.action_log_frequency << std::endl;
    
    checkDronePosition("After modifying env_config", drone, world);
    
    // Step 7: Test pathfinding (EXACTLY like original)
    std::cout << "\n--- Step 7: Test Pathfinding ---" << std::endl;
    env->setPathfindingAlgorithm("astar");
    env->setUsePathfinding(true);
    checkDronePosition("After enabling pathfinding", drone, world);
    
    // Step 8: Run first episode (EXACTLY like original)
    std::cout << "\n--- Step 8: Run First Episode ---" << std::endl;
    checkDronePosition("Before first episode", drone, world);
    
    std::cout << "\nEpisode 1" << std::endl;
    auto result = env->runEpisode(agent);
    
    std::cout << "  Steps: " << result.steps << std::endl;
    std::cout << "  Success: " << (result.success ? "Yes" : "No") << std::endl;
    std::cout << "  Total Reward: " << result.total_reward << std::endl;
    std::cout << "  Final Distance: " << result.final_distance << std::endl;
    
    checkDronePosition("After first episode", drone, world);
    
    // Show pathfinding info
    auto optimal_path = env->getOptimalPath();
    std::cout << "  Optimal Path Waypoints: " << optimal_path.size() << std::endl;
    
    // Show action execution statistics
    env->printActionStats();
    
    // Show agent debug info
    std::cout << "  Agent Debug Info:" << std::endl;
    std::cout << agent->getDebugInfo() << std::endl;
    
    std::cout << "\n=== Replicate Original Test Complete ===" << std::endl;
    return 0;
}
