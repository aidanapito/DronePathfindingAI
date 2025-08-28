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
    
    std::cout << std::setw(30) << std::left << label << ": ";
    std::cout << "Pos(" << std::fixed << std::setprecision(2) << pos.x << ", " << pos.y << ") ";
    std::cout << "H:" << std::fixed << std::setprecision(3) << heading << " ";
    std::cout << "V:" << std::fixed << std::setprecision(2) << velocity << " ";
    std::cout << "AV:" << std::fixed << std::setprecision(2) << angular_velocity << " ";
    std::cout << "Bounds:" << (in_bounds ? "YES" : "NO") << std::endl;
    
    if (!in_bounds) {
        std::cout << "  *** POSITION CORRUPTION DETECTED! ***" << std::endl;
    }
}

int main() {
    std::cout << "=== Reset Sequence Debug ===" << std::endl;
    
    // Create world with maze
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
    env->setDrone(drone);
    env->setAgent(agent);
    printDroneState("After setting components", drone, world);
    
    // Enable pathfinding
    std::cout << "\n--- Enabling Pathfinding ---" << std::endl;
    env->setPathfindingAlgorithm("astar");
    env->setUsePathfinding(true);
    printDroneState("After enabling pathfinding", drone, world);
    
    // Now let's trace the reset sequence step by step
    std::cout << "\n=== TRACING RESET SEQUENCE ===" << std::endl;
    
    // Let's manually simulate what happens in reset()
    std::cout << "\n--- Manual Reset Sequence ---" << std::endl;
    
    // Step 1: Reset drone to start position
    std::cout << "\n1. Resetting drone to start position..." << std::endl;
    printDroneState("Before reset", drone, world);
    
    sim::DroneState start_state;
    start_state.position = start_pos;
    start_state.heading = 0.0f;
    start_state.velocity = 0.0f;
    start_state.angular_velocity = 0.0f;
    
    drone->setState(start_state);
    printDroneState("After setState", drone, world);
    
    // Step 2: Reset agent
    std::cout << "\n2. Resetting agent..." << std::endl;
    printDroneState("Before agent reset", drone, world);
    
    agent->reset();
    printDroneState("After agent reset", drone, world);
    
    // Step 3: Compute optimal path
    std::cout << "\n3. Computing optimal path..." << std::endl;
    printDroneState("Before computeOptimalPath", drone, world);
    
    // Let's check what the drone position is right before pathfinding
    cv::Point2f current_pos = drone->getState().position;
    std::cout << "  Drone position before pathfinding: (" << current_pos.x << ", " << current_pos.y << ")" << std::endl;
    std::cout << "  Expected start position: (" << start_pos.x << ", " << start_pos.y << ")" << std::endl;
    std::cout << "  Positions match: " << (current_pos == start_pos ? "YES" : "NO") << std::endl;
    
    // Now let's call the actual environment reset to see if it's different
    std::cout << "\n--- Testing Actual Environment Reset ---" << std::endl;
    printDroneState("Before env->reset()", drone, world);
    
    env->reset();
    
    printDroneState("After env->reset()", drone, world);
    
    // Let's also check if the environment's internal state is corrupted
    std::cout << "\n--- Checking Environment Internal State ---" << std::endl;
    
    // Get the optimal path that was computed
    auto optimal_path = env->getOptimalPath();
    if (!optimal_path.empty()) {
        cv::Point2f first_waypoint = optimal_path[0];
        std::cout << "First waypoint in optimal path: (" << first_waypoint.x << ", " << first_waypoint.y << ")" << std::endl;
        std::cout << "Expected start position: (" << start_pos.x << ", " << start_pos.y << ")" << std::endl;
        std::cout << "Waypoint matches start: " << (first_waypoint == start_pos ? "YES" : "NO") << std::endl;
    }
    
    std::cout << "\n=== DEBUG COMPLETE ===" << std::endl;
    return 0;
}
