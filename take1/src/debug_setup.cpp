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
    std::cout << "=== Debug Setup Test ===" << std::endl;
    
    // Step 1: Create world
    std::cout << "\n--- Step 1: Create World ---" << std::endl;
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
    
    // Step 3: Set world on drone
    std::cout << "\n--- Step 3: Set World on Drone ---" << std::endl;
    drone->setWorld(world.get());
    checkDronePosition("After setWorld", drone, world);
    
    // Step 4: Create agent
    std::cout << "\n--- Step 4: Create Agent ---" << std::endl;
    agent::AgentConfig agent_config;
    agent_config.use_vision = false;
    agent_config.learning_rate = 0.1f;
    agent_config.discount_factor = 0.99f;
    agent_config.epsilon = 0.1f;
    agent_config.replay_buffer_size = 1000;
    
    auto agent = std::make_shared<agent::QLearningAgent>(agent_config);
    checkDronePosition("After agent creation", drone, world);
    
    // Step 5: Create environment
    std::cout << "\n--- Step 5: Create Environment ---" << std::endl;
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
    
    // Step 6: Set world on environment
    std::cout << "\n--- Step 6: Set World on Environment ---" << std::endl;
    env->setWorld(world);
    checkDronePosition("After setWorld on env", drone, world);
    
    // Step 7: Set drone on environment
    std::cout << "\n--- Step 7: Set Drone on Environment ---" << std::endl;
    env->setDrone(drone);
    checkDronePosition("After setDrone on env", drone, world);
    
    // Step 8: Set agent on environment
    std::cout << "\n--- Step 8: Set Agent on Environment ---" << std::endl;
    env->setAgent(agent);
    checkDronePosition("After setAgent on env", drone, world);
    
    // Step 9: Set pathfinding algorithm
    std::cout << "\n--- Step 9: Set Pathfinding Algorithm ---" << std::endl;
    env->setPathfindingAlgorithm("astar");
    checkDronePosition("After setPathfindingAlgorithm", drone, world);
    
    // Step 10: Enable pathfinding
    std::cout << "\n--- Step 10: Enable Pathfinding ---" << std::endl;
    env->setUsePathfinding(true);
    checkDronePosition("After setUsePathfinding", drone, world);
    
    // Step 11: Test single step
    std::cout << "\n--- Step 11: Test Single Step ---" << std::endl;
    checkDronePosition("Before step", drone, world);
    
    env->step(agent);
    
    checkDronePosition("After step", drone, world);
    
    // Step 12: Test environment reset
    std::cout << "\n--- Step 12: Test Environment Reset ---" << std::endl;
    checkDronePosition("Before reset", drone, world);
    
    env->reset();
    
    checkDronePosition("After reset", drone, world);
    
    std::cout << "\n=== Debug Setup Complete ===" << std::endl;
    return 0;
}
