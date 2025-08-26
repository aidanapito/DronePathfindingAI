#include "Simulator.h"
#include "agent/QLearningAgent.h"
#include "agent/VisionAgent.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <limits>
#include <string>

Simulator::Simulator() 
    : paused_(false), running_(false), episode_count_(0), 
      current_map_type_(sim::MapType::MAZE), use_vision_track_(false),
      target_fps_(60.0f), time_step_(1.0f/60.0f) {
    initializeComponents();
}

void Simulator::initializeComponents() {
    // Initialize world
    world_ = std::make_shared<sim::World>(800, 600);
    world_->generateMap(current_map_type_);
    
    // Initialize drone at start position
    cv::Point2f start_pos = world_->getStartPosition();
    drone_ = std::make_shared<sim::Drone>(start_pos);
    drone_->setWorld(world_.get());
    
    // Initialize agent
    agent_ = createAgent();
    
    // Initialize environment with improved configuration
    bridge::EnvironmentConfig env_config;
    env_config.max_steps_per_episode = 1000;
    env_config.time_step = time_step_;
    env_config.goal_reward = 200.0f;           // Strong goal motivation
    env_config.collision_penalty = -100.0f;    // Strong collision avoidance
    env_config.progress_reward = 2.0f;         // Better progress tracking
    env_config.time_penalty = -0.05f;          // Balanced time penalty
    env_config.safety_margin_penalty = -0.02f;
    
    // Conservative movement settings (from our bug fix)
    env_config.throttle_scale = 0.1f;          // 10% throttle = 10 units/step
    env_config.yaw_rate_scale = 0.1f;          // 10% yaw rate = 0.1 rad/step
    env_config.enable_safety_checks = true;
    env_config.enable_action_logging = true;
    env_config.action_log_frequency = 10;
    
    environment_ = std::make_shared<bridge::Environment>(env_config);
    environment_->setWorld(world_);
    environment_->setDrone(drone_);
    environment_->setAgent(agent_);
    
    // Enable pathfinding for better navigation
    environment_->setPathfindingAlgorithm("astar");
    environment_->setUsePathfinding(true);
    
    // Initialize UI
    ui::UIConfig ui_config;
    ui_config.window_name = "Drone Pathfinding AI";
    ui_config.window_width = 1200;
    ui_config.window_height = 800;
    ui_config.show_debug_info = true;
    ui_config.show_path_trace = true;
    ui_config.show_collision_boxes = true;
    ui_config.show_reward_info = true;
    ui_config.show_fps = true;
    ui_config.pause_key = 'p';
    ui_config.manual_mode_key = 'm';
    ui_config.record_key = 'v';
    ui_config.new_map_key = 'n';
    ui_config.reset_key = 'r';
    
    ui_ = std::make_shared<ui::SimulatorUI>(ui_config);
    
    std::cout << "Simulator components initialized" << std::endl;
    std::cout << "Environment configured with conservative movement settings" << std::endl;
}

void Simulator::run() {
    std::cout << "Simulator running..." << std::endl;
    std::cout << "Learning Track: " << (use_vision_track_ ? "Vision (Track B)" : "Q-Learning (Track A)") << std::endl;
    std::cout << "Map Type: " << getMapTypeName(current_map_type_) << std::endl;
    
    // Initialize OpenCV window
    cv::namedWindow("Drone Pathfinding AI", cv::WINDOW_AUTOSIZE);
    
    // Setup first episode
    setupEpisode();
    
    // Main simulation loop
    running_ = true;
    last_frame_time_ = std::chrono::high_resolution_clock::now();
    
    while (running_) {
        // Handle user input
        handleUserInput();
        
        // Update simulation if not paused
        if (!paused_) {
            updateSimulation();
        }
        
        // Render frame
        renderFrame();
        
        // Record frame if recording
        if (ui_ && ui_->isRecording()) {
            recordFrame();
        }
        
        // Control frame rate
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(current_time - last_frame_time_);
        int target_microseconds = static_cast<int>(1000000.0f / target_fps_);
        
        if (elapsed.count() < target_microseconds) {
            int sleep_time = target_microseconds - elapsed.count();
            std::this_thread::sleep_for(std::chrono::microseconds(sleep_time));
        }
        
        last_frame_time_ = current_time;
        
        // Check for window close
        if (cv::waitKey(1) == 27) { // ESC key
            running_ = false;
        }
    }
    
    // Cleanup
    cv::destroyAllWindows();
    std::cout << "Simulation ended." << std::endl;
    std::cout << "Total episodes completed: " << episode_count_ << std::endl;
}

void Simulator::step() {
    if (!paused_ && environment_ && agent_) {
        updateSimulation();
    }
}

// Helper function to get map type name
std::string Simulator::getMapTypeName(sim::MapType map_type) {
    switch (map_type) {
        case sim::MapType::MAZE: return "Maze";
        case sim::MapType::CORRIDOR: return "Corridor";
        case sim::MapType::OPEN_FIELD: return "Open Field";
        case sim::MapType::OBSTACLE_COURSE: return "Obstacle Course";
        default: return "Unknown";
    }
}

void Simulator::reset() {
    std::cout << "Resetting simulation..." << std::endl;
    
    // Reset environment
    if (environment_) {
        environment_->reset();
    }
    
    // Generate new map
    if (world_) {
        world_->generateMap(current_map_type_);
        std::cout << "New " << getMapTypeName(current_map_type_) << " map generated" << std::endl;
    }
    
    // Reset drone to new start position
    if (drone_ && world_) {
        cv::Point2f start_pos = world_->getStartPosition();
        drone_->setState(sim::DroneState{start_pos, 0.0f, 0.0f, 0.0f});
    }
    
    // Reset agent
    if (agent_) {
        agent_->reset();
    }
    
    // Setup new episode
    setupEpisode();
}

void Simulator::setAgent(std::shared_ptr<agent::Agent> agent) {
    agent_ = agent;
}

void Simulator::setMapType(sim::MapType map_type) {
    current_map_type_ = map_type;
}

void Simulator::setLearningTrack(bool use_vision) {
    use_vision_track_ = use_vision;
}

void Simulator::pause() {
    paused_ = true;
}

void Simulator::resume() {
    paused_ = false;
}

void Simulator::togglePause() {
    paused_ = !paused_;
}

void Simulator::setupEpisode() {
    if (!environment_ || !world_ || !drone_) return;
    
    std::cout << "\n=== Starting Episode " << (episode_count_ + 1) << " ===" << std::endl;
    
    // Reset environment (this will reset drone, agent, and compute optimal path)
    environment_->reset();
    
    // Get episode info
    auto optimal_path = environment_->getOptimalPath();
    std::cout << "Optimal path computed with " << optimal_path.size() << " waypoints" << std::endl;
    
    // Print start/goal positions
    cv::Point2f start_pos = world_->getStartPosition();
    cv::Point2f goal_pos = world_->getGoalPosition();
    std::cout << "Start: (" << start_pos.x << ", " << start_pos.y << ")" << std::endl;
    std::cout << "Goal: (" << goal_pos.x << ", " << goal_pos.y << ")" << std::endl;
    std::cout << "Initial distance to goal: " << cv::norm(goal_pos - start_pos) << std::endl;
    
    episode_count_++;
}

void Simulator::handleUserInput() {
    int key = cv::waitKey(1);
    
    switch (key) {
        case 'p':
        case 'P':
            togglePause();
            std::cout << (paused_ ? "Simulation paused" : "Simulation resumed") << std::endl;
            break;
        case 'r':
        case 'R':
            reset();
            std::cout << "Simulation reset" << std::endl;
            break;
        case 'n':
        case 'N':
            if (world_) {
                world_->generateMap(current_map_type_);
                std::cout << "New map generated" << std::endl;
            }
            break;
        case 'v':
        case 'V':
            if (ui_) {
                if (ui_->isRecording()) {
                    ui_->stopRecording();
                    std::cout << "Video recording stopped" << std::endl;
                } else {
                    ui_->startRecording("recording.mp4", 30);
                    std::cout << "Video recording started" << std::endl;
                }
            }
            break;
        case 27: // ESC
            running_ = false;
            break;
    }
}

void Simulator::updateSimulation() {
    if (!environment_ || !agent_ || !drone_ || !world_) return;
    
    // Check if current episode is done
    if (environment_->isDone()) {
        // Episode completed, get results
        auto episode_result = environment_->getCurrentObservation();
        float final_distance = drone_->getDistanceToGoal(world_->getGoalPosition());
        
        std::cout << "Episode " << episode_count_ << " completed!" << std::endl;
        std::cout << "Final distance to goal: " << final_distance << std::endl;
        
        // Check if goal was reached
        if (final_distance < 20.0f) {
            std::cout << "🎯 SUCCESS: Goal reached!" << std::endl;
        } else {
            std::cout << "❌ Episode ended without reaching goal" << std::endl;
        }
        
        // Show action statistics
        environment_->printActionStats();
        
        // Show reward breakdown
        auto reward_breakdown = environment_->getRewardBreakdown();
        std::cout << "Final Reward Breakdown:" << std::endl;
        std::cout << "  Total Reward: " << reward_breakdown.total_reward << std::endl;
        std::cout << "  Goal Reward: " << reward_breakdown.goal_reward << std::endl;
        std::cout << "  Progress Reward: " << reward_breakdown.progress_reward << std::endl;
        std::cout << "  Directional Reward: " << reward_breakdown.directional_reward << std::endl;
        std::cout << "  Path Following: " << reward_breakdown.path_following_reward << std::endl;
        
        // Show agent debug info
        if (auto q_agent = std::dynamic_pointer_cast<agent::QLearningAgent>(agent_)) {
            std::cout << "Agent Debug Info:" << std::endl;
            std::cout << q_agent->getDebugInfo() << std::endl;
        }
        
        // Setup next episode
        setupEpisode();
        return;
    }
    
    // Take a single step in the environment
    environment_->step(agent_);
    
    // Print progress periodically
    static int progress_counter = 0;
    progress_counter++;
    if (progress_counter % 50 == 0) {
        float current_distance = drone_->getDistanceToGoal(world_->getGoalPosition());
        cv::Point2f current_pos = drone_->getState().position;
        
        std::cout << "Step " << progress_counter << ": ";
        std::cout << "Pos(" << current_pos.x << ", " << current_pos.y << ") ";
        std::cout << "Distance: " << current_distance << " ";
        std::cout << "In Bounds: " << (world_->isInBounds(current_pos) ? "YES" : "NO") << std::endl;
        
        // Show current reward
        float current_reward = environment_->getCurrentReward();
        std::cout << "  Current Reward: " << current_reward << std::endl;
    }
}

void Simulator::renderFrame() {
    if (!world_ || !drone_) return;
    
    // Create display buffer
    cv::Mat display;
    world_->render(display);
    
    // Draw optimal path if available
    if (environment_) {
        auto optimal_path = environment_->getOptimalPath();
        if (!optimal_path.empty()) {
            // Draw the complete path
            for (size_t i = 0; i < optimal_path.size() - 1; ++i) {
                cv::line(display, optimal_path[i], optimal_path[i + 1], 
                        cv::Scalar(0, 255, 255), 2); // Yellow path
            }
            
            // Draw waypoints
            for (size_t i = 0; i < optimal_path.size(); ++i) {
                cv::circle(display, optimal_path[i], 5, cv::Scalar(255, 255, 0), -1); // Cyan waypoints
            }
            
            // Highlight current waypoint
            int current_waypoint = environment_->getCurrentWaypointIndex();
            if (current_waypoint >= 0 && current_waypoint < static_cast<int>(optimal_path.size())) {
                cv::circle(display, optimal_path[current_waypoint], 8, cv::Scalar(0, 255, 0), 3); // Green current waypoint
            }
        }
    }
    
    // Draw drone on the display
    cv::Point2f drone_pos = drone_->getState().position;
    cv::circle(display, drone_pos, 15, cv::Scalar(255, 0, 0), -1); // Blue circle for drone
    
    // Draw drone heading
    float heading = drone_->getState().heading;
    cv::Point2f heading_end = drone_pos + cv::Point2f(20 * cos(heading), 20 * sin(heading));
    cv::arrowedLine(display, drone_pos, heading_end, cv::Scalar(0, 255, 0), 3);
    
    // Draw goal position
    cv::Point2f goal_pos = world_->getGoalPosition();
    cv::circle(display, goal_pos, 20, cv::Scalar(0, 0, 255), 3); // Red circle for goal
    
    // Add text information
    std::string episode_text = "Episode: " + std::to_string(episode_count_);
    cv::putText(display, episode_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    
    std::string status_text = paused_ ? "PAUSED" : "RUNNING";
    cv::putText(display, status_text, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, 
                paused_ ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), 2);
    
    // Show current distance to goal
    float distance_to_goal = drone_->getDistanceToGoal(goal_pos);
    std::string distance_text = "Distance: " + std::to_string(static_cast<int>(distance_to_goal));
    cv::putText(display, distance_text, cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    
    // Show current reward if available
    if (environment_) {
        float current_reward = environment_->getCurrentReward();
        std::string reward_text = "Reward: " + std::to_string(static_cast<int>(current_reward));
        cv::putText(display, reward_text, cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    }
    
    // Add UI elements
    if (ui_) {
        ui_->render(*world_, *drone_, agent_.get());
    }
    
    // Show the frame
    cv::imshow("Drone Pathfinding AI", display);
}

void Simulator::recordFrame() {
    // TODO: Implement frame recording
}

std::shared_ptr<agent::Agent> Simulator::createAgent() {
    agent::AgentConfig config;
    config.use_vision = use_vision_track_;
    config.observation_stack = 4;
    config.learning_rate = 0.001f;
    config.discount_factor = 0.99f;
    config.epsilon = 0.1f;
    config.replay_buffer_size = 10000;
    
    if (use_vision_track_) {
        return std::make_shared<agent::VisionAgent>(config);
    } else {
        return std::make_shared<agent::QLearningAgent>(config);
    }
}

void Simulator::configureEnvironment() {
    // TODO: Implement environment configuration
}

float Simulator::getAverageReward() const {
    if (!environment_) return 0.0f;
    
    // For now, return the current episode reward
    // TODO: Implement proper reward history tracking
    return environment_->getCurrentReward();
}

float Simulator::getSuccessRate() const {
    // TODO: Implement proper success rate tracking
    // For now, return a placeholder
    return 0.0f;
}
