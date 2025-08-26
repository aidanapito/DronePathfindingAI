#include <iostream>
#include <memory>
#include <chrono>
#include <thread>

#include "sim/World.h"
#include "sim/Drone.h"
#include "bridge/Environment.h"
#include "ui/SimulatorUI.h"
#include "agent/QLearningAgent.h"

using namespace std;

int main() {
    cout << "=== 3D Drone Pathfinding AI Demo ===" << endl;
    
    // Create 3D world
    auto world = std::make_shared<sim::World>(800, 600, 400);
    world->generateMap(sim::MapType::SKYSCRAPER, 42);
    
    // Add some 3D obstacles
    world->addRandomObstacles(15, 20.0f, 50.0f, 30.0f, 150.0f);
    world->addVerticalObstacles(8, 15.0f, 40.0f, 80.0f, 200.0f);
    
    // Set start and goal positions in 3D
    world->setStartPosition(cv::Point3f(50, 50, 50));
    world->setGoalPosition(cv::Point3f(700, 500, 100));
    
    // Create 3D drone
    auto drone = std::make_shared<sim::Drone>(cv::Point3f(50, 50, 50), 0.0f, 0.0f, 0.0f);
    drone->setWorld(world.get());
    
    // Configure 3D environment
    bridge::EnvironmentConfig config;
    config.max_steps_per_episode = 1000;
    config.time_step = 0.1f;
    config.render_episodes = true;
    config.enable_3d_mode = true;
    config.enable_3d_pathfinding = true;
    config.max_altitude = 300.0f;
    config.min_altitude = 10.0f;
    config.altitude_safety_margin = 15.0f;
    
    // Scale factors for 3D actions
    config.throttle_scale = 1.0f;
    config.yaw_rate_scale = 1.0f;
    config.pitch_rate_scale = 1.0f;
    config.roll_rate_scale = 1.0f;
    config.vertical_thrust_scale = 1.0f;
    
    // Create environment
    auto env = std::make_shared<bridge::Environment>(config);
    env->setWorld(world);
    env->setDrone(drone);
    env->set3DMode(true);
    env->set3DPathfinding(true);
    
    // Create 3D agent
    agent::AgentConfig agent_config;
    agent_config.use_vision = false;
    agent_config.use_3d = true;
    agent_config.learning_rate = 0.001f;
    agent_config.discount_factor = 0.99f;
    agent_config.epsilon = 0.1f;
    agent_config.max_altitude = 300.0f;
    agent_config.min_altitude = 10.0f;
    agent_config.enable_3d_pathfinding = true;
    
    auto agent = std::make_shared<agent::QLearningAgent>(agent_config);
    env->setAgent(agent);
    
    // Create 3D UI
    auto ui = std::make_shared<ui::SimulatorUI>();
    ui->createWindow("3D Drone Simulator", 1200, 800);
    ui->setRenderMode("3D");
    ui->setShowGrid(true);
    ui->setShowAxes(true);
    ui->setShowPath(true);
    ui->setShowObstacles(true);
    ui->setShowDrone(true);
    ui->setShowGoal(true);
    
    // Set up 3D camera
    ui::Camera3D camera;
    camera.position = cv::Point3f(400, 300, 250);
    camera.target = cv::Point3f(400, 300, 100);
    camera.up = cv::Point3f(0, 1, 0);
    ui->setCamera(camera);
    
    cout << "Starting 3D simulation..." << endl;
    cout << "Controls:" << endl;
    cout << "  Mouse drag: Rotate camera" << endl;
    cout << "  Mouse wheel: Zoom" << endl;
    cout << "  Right mouse drag: Pan camera" << endl;
    cout << "  R: Reset camera" << endl;
    cout << "  Space: Pause/Resume" << endl;
    cout << "  ESC: Exit" << endl;
    
    // Main simulation loop
    bool running = true;
    bool paused = false;
    
    while (running) {
        // Handle events
        ui->handleMouseEvents();
        ui->handleKeyboardEvents();
        
        if (!paused) {
            // Run one step
            env->step(agent);
            
            // Check if episode is done
            if (env->isDone()) {
                cout << "Episode completed!" << endl;
                env->reset();
            }
        }
        
        // Get current paths for visualization
        auto path_3d = env->getOptimalPath3D();
        auto path_2d = env->getOptimalPath();
        
        // Render 3D scene
        ui->render3D(*world, *drone, path_3d, path_2d);
        ui->updateWindow();
        
        // Small delay for smooth animation
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        // Check for exit
        char key = cv::waitKey(1);
        if (key == 27) { // ESC
            running = false;
        } else if (key == 32) { // Space
            paused = !paused;
            cout << (paused ? "Paused" : "Resumed") << endl;
        } else if (key == 'r' || key == 'R') {
            ui->resetCamera();
        }
    }
    
    cout << "Simulation ended." << endl;
    
    // Print final statistics
    cout << "\n=== Final Statistics ===" << endl;
    cout << "Episodes completed: " << env->getEpisodeCount() << endl;
    cout << "Success rate: " << (env->getSuccessRate() * 100.0f) << "%" << endl;
    cout << "Average episode length: " << env->getAverageEpisodeLength() << " steps" << endl;
    cout << "Average altitude error: " << env->getAverageAltitudeError() << " units" << endl;
    
    // Print action statistics
    env->printActionStats();
    
    // Print reward breakdown
    auto reward_breakdown = env->getRewardBreakdown();
    cout << "\n=== Reward Breakdown ===" << endl;
    cout << "Goal reward: " << reward_breakdown.goal_reward << endl;
    cout << "Altitude reward: " << reward_breakdown.altitude_reward << endl;
    cout << "Vertical progress: " << reward_breakdown.vertical_progress_reward << endl;
    cout << "Clearance reward: " << reward_breakdown.clearance_reward << endl;
    cout << "Total reward: " << reward_breakdown.total_reward << endl;
    
    ui->closeWindow();
    return 0;
}
