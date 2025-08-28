#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "bridge/Environment.h"
#include "sim/World.h"
#include "sim/Drone.h"
#include "agent/QLearningAgent.h"
#include "ui/SimulatorUI.h"

int main() {
    std::cout << "=== Interactive 3D Drone Viewer ===" << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  Mouse drag: Orbit camera" << std::endl;
    std::cout << "  Mouse wheel: Zoom camera" << std::endl;
    std::cout << "  ESC: Exit" << std::endl;
    std::cout << "  SPACE: Pause/Resume" << std::endl;
    std::cout << "  R: Reset camera" << std::endl;
    std::cout << std::endl;
    
    // Create 3D world
    auto world = std::make_shared<sim::World>(800, 600, 400);
    world->generateMap(sim::MapType::SKYSCRAPER);
    
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
    
    // Very slow movement settings for better 3D viewing
    config.throttle_scale = 0.02f;          // 2% throttle = 2 units/step (much slower)
    config.yaw_rate_scale = 0.05f;          // 5% yaw rate = 0.05 rad/step (slower turning)
    config.pitch_rate_scale = 0.05f;        // 3D pitch rate scaling (slower)
    config.roll_rate_scale = 0.05f;         // 3D roll rate scaling (slower)
    config.vertical_thrust_scale = 0.05f;   // 3D vertical thrust scaling (slower)
    
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
    
    // Create UI
    auto ui = std::make_shared<ui::SimulatorUI>();
    ui->createWindow("3D Drone Navigation", 1200, 800);
    
    // Set display options
    ui->setShowGrid(true);
    ui->setShowAxes(true);
    ui->setShowPath(true);
    ui->setShowObstacles(true);
    ui->setShowDrone(true);
    ui->setShowGoal(true);
    
    // Set 3D render mode
    ui->setRenderMode("3D");
    
    // Get optimal path
    env->setPathfindingAlgorithm("astar");
    env->setUsePathfinding(true);
    env->reset();
    auto optimal_path_3d = env->getOptimalPath3D();
    
    std::cout << "3D A* path found with " << optimal_path_3d.size() << " waypoints" << std::endl;
    
    // Main visualization loop
    bool running = true;
    bool paused = false;
    int step = 0;
    
    while (running) {
        // Handle key events
        int key = cv::waitKey(1) & 0xFF;
        
        switch (key) {
            case 27: // ESC
                running = false;
                break;
            case 32: // SPACE
                paused = !paused;
                std::cout << (paused ? "Paused" : "Resumed") << std::endl;
                break;
            case 'r':
            case 'R':
                ui->resetCamera();
                std::cout << "Camera reset" << std::endl;
                break;
        }
        
        if (!paused) {
            // Run one step of the episode
            if (step < 100) { // Limit steps for demo
                env->step(agent);
                step++;
                
                if (step % 10 == 0) {
                    float distance_to_goal = drone->getDistanceToGoal(world->getGoalPosition());
                    std::cout << "Step " << step << ": Distance to goal: " 
                              << distance_to_goal << std::endl;
                }
            }
        }
        
        // Render 3D scene
        ui->render3D(*world, *drone, optimal_path_3d, {});
        
        // Update window
        ui->updateWindow();
        
        // Small delay for smooth animation
        cv::waitKey(50);
    }
    
    // Cleanup
    ui->closeWindow();
    
    std::cout << "\n=== Interactive 3D Viewer Closed ===" << std::endl;
    return 0;
}
