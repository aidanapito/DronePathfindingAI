#include <iostream>
#include <memory>
#include <iomanip>
#include "bridge/Environment.h"
#include "agent/QLearningAgent.h"
#include "sim/World.h"
#include "sim/Drone.h"

void printDroneState(const std::string& label, const std::shared_ptr<sim::Drone>& drone, const std::shared_ptr<sim::World>& world) {
    cv::Point2f pos = drone->getState().position;
    float heading = drone->getState().heading;
    float velocity = drone->getState().velocity;
    float angular_velocity = drone->getState().angular_velocity;
    bool in_bounds = world->isInBounds(pos);
    
    std::cout << std::setw(20) << std::left << label << ": ";
    std::cout << "Pos(" << std::fixed << std::setprecision(2) << pos.x << ", " << pos.y << ") ";
    std::cout << "H:" << std::fixed << std::setprecision(3) << heading << " ";
    std::cout << "V:" << std::fixed << std::setprecision(2) << velocity << " ";
    std::cout << "AV:" << std::fixed << std::setprecision(2) << angular_velocity << " ";
    std::cout << "Bounds:" << (in_bounds ? "YES" : "NO") << std::endl;
    
    if (!in_bounds) {
        std::cout << "  *** POSITION CORRUPTION DETECTED! ***" << std::endl;
    }
}

void printStepInfo(int step, const std::shared_ptr<sim::Drone>& drone, const std::shared_ptr<sim::World>& world) {
    std::cout << "\n--- Step " << step << " ---" << std::endl;
    printDroneState("Before step", drone, world);
}

int main() {
    std::cout << "=== Episode Loop Bug Debug ===" << std::endl;
    
    // Create world with maze (exactly like failing test)
    std::cout << "\n--- Creating World ---" << std::endl;
    auto world = std::make_shared<sim::World>(800, 600);
    world->generateMap(sim::MapType::MAZE);
    std::cout << "World created: " << world->getSize().width << "x" << world->getSize().height << std::endl;
    
    cv::Point2f start_pos = world->getStartPosition();
    cv::Point2f goal_pos = world->getGoalPosition();
    std::cout << "Start: (" << start_pos.x << ", " << start_pos.y << ")" << std::endl;
    std::cout << "Goal: (" << goal_pos.x << ", " << goal_pos.y << ")" << std::endl;
    
    // Create drone
    std::cout << "\n--- Creating Drone ---" << std::endl;
    auto drone = std::make_shared<sim::Drone>(start_pos, 0.0f);
    drone->setWorld(world.get());
    printDroneState("After creation", drone, world);
    
    // Create agent
    std::cout << "\n--- Creating Agent ---" << std::endl;
    agent::AgentConfig agent_config;
    agent_config.use_vision = false;
    agent_config.learning_rate = 0.1f;
    agent_config.discount_factor = 0.99f;
    agent_config.epsilon = 0.1f;
    agent_config.replay_buffer_size = 1000;
    
    auto agent = std::make_shared<agent::QLearningAgent>(agent_config);
    printDroneState("After agent creation", drone, world);
    
    // Create environment
    std::cout << "\n--- Creating Environment ---" << std::endl;
    bridge::EnvironmentConfig env_config;
    env_config.max_steps_per_episode = 1000;
    env_config.time_step = 0.1f;
    env_config.render_episodes = true;
    env_config.goal_reward = 100.0f;
    env_config.collision_penalty = -50.0f;
    env_config.progress_reward = 1.0f;
    env_config.time_penalty = -0.1f;
    
    auto env = std::make_shared<bridge::Environment>(env_config);
    printDroneState("After environment creation", drone, world);
    
    // Set world, drone, and agent on environment
    std::cout << "\n--- Setting Environment Components ---" << std::endl;
    env->setWorld(world);
    printDroneState("After setWorld", drone, world);
    
    env->setDrone(drone);
    printDroneState("After setDrone", drone, world);
    
    env->setAgent(agent);
    printDroneState("After setAgent", drone, world);
    
    // Enable pathfinding
    std::cout << "\n--- Enabling Pathfinding ---" << std::endl;
    env->setPathfindingAlgorithm("astar");
    printDroneState("After setPathfindingAlgorithm", drone, world);
    
    env->setUsePathfinding(true);
    printDroneState("After setUsePathfinding", drone, world);
    
    // Now let's trace the episode execution step by step
    std::cout << "\n=== STARTING EPISODE EXECUTION TRACE ===" << std::endl;
    
    // Get the environment's internal state before episode
    printDroneState("Before episode starts", drone, world);
    
    // Let's manually trace a few steps to see where corruption occurs
    std::cout << "\n--- Manual Step-by-Step Trace ---" << std::endl;
    
    for (int step = 0; step < 10; ++step) {
        printStepInfo(step, drone, world);
        
        // Check if we're already corrupted
        if (!world->isInBounds(drone->getState().position)) {
            std::cout << "  *** DRONE ALREADY CORRUPTED BEFORE STEP " << step << " ***" << std::endl;
            break;
        }
        
        // Take a single step
        env->step(agent);
        
        // Check position after step
        printDroneState("After step", drone, world);
        
        // If corrupted, this is where it happened
        if (!world->isInBounds(drone->getState().position)) {
            std::cout << "  *** CORRUPTION OCCURRED DURING STEP " << step << " ***" << std::endl;
            std::cout << "  *** The bug is in Environment::step() ***" << std::endl;
            break;
        }
        
        std::cout << std::endl;
    }
    
    // If we get here, let's check what the episode result would be
    std::cout << "\n--- Episode Result Check ---" << std::endl;
    printDroneState("Final drone state", drone, world);
    
    // Let's also check if the environment reset would fix it
    std::cout << "\n--- Testing Environment Reset ---" << std::endl;
    printDroneState("Before reset", drone, world);
    
    env->reset();
    
    printDroneState("After reset", drone, world);
    
    std::cout << "\n=== DEBUG COMPLETE ===" << std::endl;
    return 0;
}
