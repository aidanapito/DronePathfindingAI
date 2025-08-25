#include "Simulator.h"
#include "agent/QLearningAgent.h"
#include "agent/VisionAgent.h"
#include <iostream>
#include <thread>
#include <chrono>

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
    
    // Initialize environment
    bridge::EnvironmentConfig env_config;
    env_config.max_steps_per_episode = 1000;
    env_config.time_step = time_step_;
    env_config.goal_reward = 1.0f;
    env_config.collision_penalty = -1.0f;
    env_config.progress_reward = 0.02f;
    env_config.time_penalty = -0.005f;
    env_config.safety_margin_penalty = -0.02f;
    
    environment_ = std::make_shared<bridge::Environment>(env_config);
    environment_->setWorld(world_);
    environment_->setDrone(drone_);
    environment_->setAgent(agent_);
    
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
}

void Simulator::run() {
    std::cout << "Simulator running..." << std::endl;
    
    // Initialize OpenCV window
    cv::namedWindow("Drone Pathfinding AI", cv::WINDOW_AUTOSIZE);
    
    // Main simulation loop
    running_ = true;
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
}

void Simulator::step() {
    // TODO: Implement single simulation step
}

void Simulator::reset() {
    if (environment_) {
        environment_->reset();
    }
    
    if (world_) {
        world_->generateMap(current_map_type_);
    }
    
    if (drone_) {
        cv::Point2f start_pos = world_->getStartPosition();
        drone_->setState(sim::DroneState{start_pos, 0.0f, 0.0f, 0.0f});
    }
    
    if (agent_) {
        agent_->reset();
    }
    
    episode_count_++;
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
    // TODO: Implement episode setup
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
    if (world_) {
        world_->update(time_step_);
    }
    
    if (drone_) {
        // Simple goal-seeking behavior
        cv::Point2f drone_pos = drone_->getState().position;
        cv::Point2f goal_pos = world_->getGoalPosition();
        float current_heading = drone_->getState().heading;
        
        // Calculate direction to goal
        cv::Point2f to_goal = goal_pos - drone_pos;
        float goal_distance = cv::norm(to_goal);
        float goal_angle = atan2(to_goal.y, to_goal.x);
        
        // Calculate heading difference (normalize to [-π, π])
        float heading_diff = goal_angle - current_heading;
        while (heading_diff > M_PI) heading_diff -= 2 * M_PI;
        while (heading_diff < -M_PI) heading_diff += 2 * M_PI;
        
        // Check for obstacles ahead with multiple detection points
        bool obstacle_ahead = false;
        bool obstacle_near = false;
        
        // Far detection point (for early warning)
        float far_look_ahead = 60.0f;  // Increased from 30.0f
        cv::Point2f far_look_ahead_pos = drone_pos + cv::Point2f(
            far_look_ahead * cos(current_heading),
            far_look_ahead * sin(current_heading)
        );
        
        // Medium detection point
        float medium_look_ahead = 40.0f;
        cv::Point2f medium_look_ahead_pos = drone_pos + cv::Point2f(
            medium_look_ahead * cos(current_heading),
            medium_look_ahead * sin(current_heading)
        );
        
        // Near detection point (for immediate action)
        float near_look_ahead = 25.0f;
        cv::Point2f near_look_ahead_pos = drone_pos + cv::Point2f(
            near_look_ahead * cos(current_heading),
            near_look_ahead * sin(current_heading)
        );
        
        // Check collision at different distances
        if (world_->checkCollision(far_look_ahead_pos, 8.0f)) {
            obstacle_ahead = true;
        } else if (world_->checkCollision(medium_look_ahead_pos, 6.0f)) {
            obstacle_ahead = true;
        } else if (world_->checkCollision(near_look_ahead_pos, 4.0f)) {
            obstacle_near = true;
        }
        
        // Check for obstacles on the sides to help with navigation
        bool obstacle_left = false;
        bool obstacle_right = false;
        
        // Check left side
        cv::Point2f left_check_pos = drone_pos + cv::Point2f(
            20.0f * cos(current_heading + M_PI/2.0f),
            20.0f * sin(current_heading + M_PI/2.0f)
        );
        if (world_->checkCollision(left_check_pos, 6.0f)) {
            obstacle_left = true;
        }
        
        // Check right side
        cv::Point2f right_check_pos = drone_pos + cv::Point2f(
            20.0f * cos(current_heading - M_PI/2.0f),
            20.0f * sin(current_heading - M_PI/2.0f)
        );
        if (world_->checkCollision(right_check_pos, 6.0f)) {
            obstacle_right = true;
        }
        
        // Decision making
        float throttle = 0.0f;
        float yaw_rate = 0.0f;
        
        if (obstacle_ahead) {
            // Obstacle detected far ahead - start turning early
            if (obstacle_right) {
                // Right side blocked, turn left
                yaw_rate = -1.2f;
            } else {
                // Turn right to avoid
                yaw_rate = 1.2f;
            }
            throttle = 0.0f;  // Stop forward movement
        } else if (obstacle_near) {
            // Obstacle detected nearby - turn more aggressively
            if (obstacle_right) {
                // Right side blocked, turn left
                yaw_rate = -1.5f;
            } else {
                // Turn right more aggressively
                yaw_rate = 1.5f;
            }
            throttle = 0.0f;  // Stop forward movement
        } else if (std::abs(heading_diff) > 0.3f) {
            // Not facing goal - turn towards it
            yaw_rate = (heading_diff > 0) ? 0.8f : -0.8f;
            throttle = 0.0f;  // Don't move forward while turning
        } else {
            // Facing goal and no obstacles - move forward
            throttle = 0.8f;  // Move forward at 80% speed
            yaw_rate = 0.0f;  // No turning needed
        }
        
        // Apply emergency stop if too close to obstacles
        if (drone_->isEmergencyStop()) {
            throttle = 0.0f;
            yaw_rate = 0.0f;
        }
        
        // Update drone
        drone_->update(time_step_, throttle, yaw_rate);
        
        // Check if goal reached
        if (goal_distance < 20.0f) {
            std::cout << "Goal reached! Episode completed successfully!" << std::endl;
            reset();
        }
        
        // Check if drone is stuck (not moving for too long)
        static int stuck_counter = 0;
        if (std::abs(drone_->getState().velocity) < 1.0f) {
            stuck_counter++;
            if (stuck_counter > 100) { // Stuck for 100 frames
                std::cout << "Drone appears stuck, resetting..." << std::endl;
                reset();
                stuck_counter = 0;
            }
        } else {
            stuck_counter = 0;
        }
    }
}

void Simulator::renderFrame() {
    if (!world_ || !drone_) return;
    
    // Create display buffer
    cv::Mat display;
    world_->render(display);
    
    // Draw drone on the display
    cv::Point2f drone_pos = drone_->getState().position;
    cv::circle(display, drone_pos, 15, cv::Scalar(255, 0, 0), -1); // Blue circle for drone
    
    // Draw drone heading
    float heading = drone_->getState().heading;
    cv::Point2f heading_end = drone_pos + cv::Point2f(20 * cos(heading), 20 * sin(heading));
    cv::arrowedLine(display, drone_pos, heading_end, cv::Scalar(0, 255, 0), 3);
    
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
    // TODO: Implement average reward calculation
    return 0.0f;
}

float Simulator::getSuccessRate() const {
    // TODO: Implement success rate calculation
    return 0.0f;
}
