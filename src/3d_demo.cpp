#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include "bridge/Environment.h"
#include "sim/World.h"
#include "sim/Drone.h"
#include "agent/QLearningAgent.h"

int main() {
    std::cout << "=== 3D Drone Pathfinding Demo ===" << std::endl;
    
    // Create 3D world
    auto world = std::make_shared<sim::World>(800, 600, 400);
    
    // Test different 3D map types
    std::vector<sim::MapType> map_types = {
        sim::MapType::SKYSCRAPER,
        sim::MapType::UNDERWATER,
        sim::MapType::MOUNTAIN_PASS
    };
    
    for (auto map_type : map_types) {
        std::cout << "\n--- Testing " << (map_type == sim::MapType::SKYSCRAPER ? "Skyscraper" :
                                        map_type == sim::MapType::UNDERWATER ? "Underwater" : "Mountain Pass") 
                  << " Map ---" << std::endl;
        
        // Generate 3D map
        world->generateMap(map_type);
        
        // Create drone
        auto drone = std::make_shared<sim::Drone>(cv::Point3f(50.0f, 300.0f, 200.0f));
        
        // Create environment configuration
        bridge::EnvironmentConfig config;
        config.max_steps_per_episode = 1000;
        config.time_step = 0.1f;
        config.render_episodes = true;
        config.enable_3d_mode = true;
        config.enable_3d_pathfinding = true;
        config.max_altitude = 350.0f;
        config.min_altitude = 50.0f;
        config.altitude_safety_margin = 20.0f;
        config.goal_reward = 1000.0f;
        config.collision_penalty = -500.0f;
        config.progress_reward = 10.0f;
        config.time_penalty = -1.0f;
        config.safety_margin_penalty = -10.0f;
        
        // Create environment
        auto env = std::make_shared<bridge::Environment>(config);
        env->setWorld(world);
        env->setDrone(drone);
        
        // Create Q-learning agent
        agent::AgentConfig agent_config;
        agent_config.use_vision = false;
        agent_config.learning_rate = 0.1f;
        agent_config.discount_factor = 0.9f;
        agent_config.epsilon = 0.1f;
        
        auto agent = std::make_shared<agent::QLearningAgent>(agent_config);
        env->setAgent(agent);
        
        // Test 3D pathfinding
        std::cout << "Testing 3D A* pathfinding..." << std::endl;
        env->setPathfindingAlgorithm("astar");
        env->setUsePathfinding(true);
        
        // Reset environment to compute initial path
        env->reset();
        
        // Get optimal 3D path
        auto optimal_path_3d = env->getOptimalPath3D();
        std::cout << "3D A* path found with " << optimal_path_3d.size() << " waypoints" << std::endl;
        
        // Test 3D flood fill pathfinding
        std::cout << "Testing 3D Flood Fill pathfinding..." << std::endl;
        env->setPathfindingAlgorithm("floodfill");
        env->reset();
        
        auto flood_fill_path_3d = env->getOptimalPath3D();
        std::cout << "3D Flood Fill path found with " << flood_fill_path_3d.size() << " waypoints" << std::endl;
        
        // Test collision detection in 3D
        std::cout << "Testing 3D collision detection..." << std::endl;
        cv::Point3f test_pos(400, 300, 200);
        bool collision = world->checkCollision(test_pos, 10.0f);
        std::cout << "Collision at (" << test_pos.x << ", " << test_pos.y << ", " << test_pos.z 
                  << "): " << (collision ? "YES" : "NO") << std::endl;
        
        // Test bounds checking in 3D
        bool in_bounds = world->isInBounds(test_pos);
        std::cout << "Position in bounds: " << (in_bounds ? "YES" : "NO") << std::endl;
        
        // Test 3D rendering
        std::cout << "Testing 3D rendering..." << std::endl;
        cv::Mat render_output;
        cv::Point3f camera_pos(400, 300, 500);
        cv::Point3f camera_target(400, 300, 200);
        world->render3D(render_output, camera_pos, camera_target);
        
        // Save 3D render
        std::string filename = "3d_render_" + 
                              std::string(map_type == sim::MapType::SKYSCRAPER ? "skyscraper" :
                               map_type == sim::MapType::UNDERWATER ? "underwater" : "mountain") + ".png";
        cv::imwrite(filename, render_output);
        std::cout << "3D render saved as " << filename << std::endl;
        
        // Test a short episode
        std::cout << "Running short 3D episode..." << std::endl;
        auto result = env->runEpisode(agent);
        
        std::cout << "Episode completed:" << std::endl;
        std::cout << "  Steps: " << result.steps << std::endl;
        std::cout << "  Success: " << (result.success ? "YES" : "NO") << std::endl;
        std::cout << "  Total Reward: " << result.total_reward << std::endl;
        std::cout << "  Final Distance: " << result.final_distance << std::endl;
        std::cout << "  Path Length: " << result.path_trace.size() << std::endl;
        
        // Print action statistics
        env->printActionStats();
        
        // Print reward breakdown
        auto reward_breakdown = env->getRewardBreakdown();
        std::cout << "\nReward Breakdown:" << std::endl;
        std::cout << "  Goal Reward: " << reward_breakdown.goal_reward << std::endl;
        std::cout << "  Collision Penalty: " << reward_breakdown.collision_penalty << std::endl;
        std::cout << "  Progress Reward: " << reward_breakdown.progress_reward << std::endl;
        std::cout << "  Path Following Reward: " << reward_breakdown.path_following_reward << std::endl;
        std::cout << "  Altitude Reward: " << reward_breakdown.altitude_reward << std::endl;
        std::cout << "  Clearance Reward: " << reward_breakdown.clearance_reward << std::endl;
        std::cout << "  Total Reward: " << reward_breakdown.total_reward << std::endl;
        
        std::cout << "\n" << std::string(50, '-') << std::endl;
    }
    
    std::cout << "\n=== 3D Demo Completed ===" << std::endl;
    std::cout << "Check the generated PNG files for 3D renders of each map type." << std::endl;
    
    return 0;
}
