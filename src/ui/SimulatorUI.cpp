#include "ui/SimulatorUI.h"
#include "bridge/Environment.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>

// M_PI is already defined on macOS, so only define if not present
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ui {

SimulatorUI::SimulatorUI(const UIConfig& config) 
    : config_(config), current_mode_(UIMode::SIMULATION), 
      show_debug_overlay_(false), current_fps_(0.0f), 
      last_frame_time_(std::chrono::high_resolution_clock::now()) {
    
    // Initialize display buffer
    display_buffer_ = cv::Mat::zeros(config.window_height, config.window_width, CV_8UC3);
    
    std::cout << "UI initialized: " << config.window_width << "x" << config.window_height << std::endl;
}

void SimulatorUI::render(const sim::World& world, const sim::Drone& drone, 
                         const agent::Agent* agent) {
    // Clear the display buffer
    display_buffer_ = cv::Mat::zeros(config_.window_height, config_.window_width, CV_8UC3);
    
    // Render world (this will be called from the main simulator)
    // renderWorld(world);
    
    // Render drone
    renderDrone(drone);
    
    // Render path trace if enabled
    if (config_.show_path_trace) {
        renderPathTrace();
    }
    
    // Render collision boxes if enabled
    if (config_.show_collision_boxes) {
        renderCollisionBoxes(world);
    }
    
    // Render FPS if enabled
    if (config_.show_fps) {
        renderFPS();
    }
    
    // Update FPS calculation
    auto current_time = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(current_time - last_frame_time_);
    if (elapsed.count() > 0) {
        current_fps_ = 1000000.0f / elapsed.count();
    }
    last_frame_time_ = current_time;
}

void SimulatorUI::renderUI(const bridge::Environment& env) {
    if (config_.show_debug_info) {
        renderDebugInfo(env);
    }
    
    if (config_.show_reward_info) {
        float current_reward = env.getCurrentReward();
        renderRewardInfo(current_reward);
    }
}

bool SimulatorUI::handleKeyPress(int key) {
    switch (key) {
        case 'p':
        case 'P':
            handlePauseKey();
            return true;
        case 'm':
        case 'M':
            handleManualModeKey();
            return true;
        case 'v':
        case 'V':
            handleRecordKey();
            return true;
        case 'n':
        case 'N':
            handleNewMapKey();
            return true;
        case 'r':
        case 'R':
            handleResetKey();
            return true;
        default:
            return false;
    }
}

void SimulatorUI::handleMouseEvent(int event, int x, int y, int flags) {
    // Handle mouse events for interactive features
    switch (event) {
        case cv::EVENT_LBUTTONDOWN:
            // Left click - could be used for setting waypoints or selecting objects
            std::cout << "Mouse click at (" << x << ", " << y << ")" << std::endl;
            break;
        case cv::EVENT_RBUTTONDOWN:
            // Right click - could be used for context menu
            std::cout << "Right click at (" << x << ", " << y << ")" << std::endl;
            break;
        case cv::EVENT_MOUSEMOVE:
            // Mouse movement - could be used for hover effects
            break;
    }
}

void SimulatorUI::setMode(UIMode mode) {
    current_mode_ = mode;
}

void SimulatorUI::startRecording(const std::string& output_path, int fps) {
    if (video_writer_.isOpened()) {
        stopRecording();
    }
    
    // Create video writer
    int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
    bool success = video_writer_.open(output_path, fourcc, fps, 
                                     cv::Size(config_.window_width, config_.window_height));
    
    if (success) {
        current_mode_ = UIMode::RECORDING;
        std::cout << "Started recording to: " << output_path << " at " << fps << " FPS" << std::endl;
    } else {
        std::cerr << "Failed to start recording to: " << output_path << std::endl;
    }
}

void SimulatorUI::stopRecording() {
    if (video_writer_.isOpened()) {
        video_writer_.release();
        current_mode_ = UIMode::SIMULATION;
        std::cout << "Recording stopped" << std::endl;
    }
}

void SimulatorUI::setConfig(const UIConfig& config) {
    config_ = config;
}

void SimulatorUI::renderWorld(const sim::World& world) {
    // TODO: Implement world rendering
}

void SimulatorUI::renderDrone(const sim::Drone& drone) {
    auto state = drone.getState();
    cv::Point2f drone_pos = worldToScreen(state.position);
    
    // Draw drone body (blue circle)
    cv::circle(display_buffer_, drone_pos, 20, cv::Scalar(255, 0, 0), -1);
    
    // Draw drone border (white outline)
    cv::circle(display_buffer_, drone_pos, 20, cv::Scalar(255, 255, 255), 2);
    
    // Draw drone heading (green arrow)
    float heading = state.heading;
    cv::Point2f heading_end = drone_pos + cv::Point2f(30 * cos(heading), 30 * sin(heading));
    drawArrow(display_buffer_, drone_pos, heading_end, cv::Scalar(0, 255, 0), 3);
    
    // Draw drone status indicators
    if (drone.isEmergencyStop()) {
        // Red warning circle for emergency stop
        cv::circle(display_buffer_, drone_pos, 25, cv::Scalar(0, 0, 255), 3);
    }
    
    // Draw velocity indicator (small arrow showing movement direction)
    if (std::abs(state.velocity) > 0.1f) {
        float velocity_angle = atan2(state.velocity, 0.0f); // Assuming forward velocity
        cv::Point2f vel_end = drone_pos + cv::Point2f(15 * cos(velocity_angle), 15 * sin(velocity_angle));
        drawArrow(display_buffer_, drone_pos, vel_end, cv::Scalar(255, 255, 0), 2);
    }
    
    // Draw drone label
    std::string drone_label = "DRONE";
    cv::Point label_pos(drone_pos.x - 25, drone_pos.y - 30);
    drawText(display_buffer_, drone_label, label_pos, cv::Scalar(255, 255, 255), 0.5);
}

void SimulatorUI::renderPathTrace() {
    if (path_trace_.size() < 2) return;
    
    // Draw the complete path trace
    for (size_t i = 0; i < path_trace_.size() - 1; ++i) {
        cv::Point2f screen_start = worldToScreen(path_trace_[i]);
        cv::Point2f screen_end = worldToScreen(path_trace_[i + 1]);
        
        // Use different colors based on path age (fade from bright to dim)
        float alpha = static_cast<float>(i) / path_trace_.size();
        cv::Scalar color(255 * alpha, 255 * alpha, 255 * alpha);
        
        cv::line(display_buffer_, screen_start, screen_end, color, 1);
    }
    
    // Draw path trace points
    for (size_t i = 0; i < path_trace_.size(); i += 5) { // Every 5th point to avoid clutter
        cv::Point2f screen_pos = worldToScreen(path_trace_[i]);
        cv::circle(display_buffer_, screen_pos, 2, cv::Scalar(200, 200, 200), -1);
    }
}

// Add method to update path trace
void SimulatorUI::addPathPoint(const cv::Point2f& world_pos) {
    path_trace_.push_back(world_pos);
    
    // Limit path trace length to prevent memory issues
    if (path_trace_.size() > 1000) {
        path_trace_.erase(path_trace_.begin());
    }
}

void SimulatorUI::renderDebugInfo(const bridge::Environment& env) {
    // Create debug info panel (top-left corner)
    cv::Point panel_start(10, 10);
    cv::Point panel_end(350, 200);
    
    // Draw semi-transparent background
    cv::Mat overlay = display_buffer_.clone();
    cv::rectangle(overlay, panel_start, panel_end, cv::Scalar(0, 0, 0), -1);
    cv::addWeighted(display_buffer_, 0.7, overlay, 0.3, 0, display_buffer_);
    
    // Draw panel border
    cv::rectangle(display_buffer_, panel_start, panel_end, cv::Scalar(255, 255, 255), 2);
    
    // Render debug information
    int y_offset = 30;
    int line_height = 20;
    
    // Title
    drawText(display_buffer_, "DEBUG INFO", cv::Point(20, y_offset), cv::Scalar(255, 255, 0), 0.8);
    y_offset += line_height + 5;
    
    // Episode info
    std::string episode_info = "Episode: " + std::to_string(env.getEpisodeCount());
    drawText(display_buffer_, episode_info, cv::Point(20, y_offset), cv::Scalar(255, 255, 255), 0.6);
    y_offset += line_height;
    
    // Current step
    std::string step_info = "Step: " + std::to_string(env.getCurrentStep());
    drawText(display_buffer_, step_info, cv::Point(20, y_offset), cv::Scalar(255, 255, 255), 0.6);
    y_offset += line_height;
    
    // Current reward
    float current_reward = env.getCurrentReward();
    std::string reward_info = "Reward: " + std::to_string(current_reward);
    cv::Scalar reward_color = (current_reward > 0) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    drawText(display_buffer_, reward_info, cv::Point(20, y_offset), reward_color, 0.6);
    y_offset += line_height;
    
    // Pathfinding info
    if (env.getUsePathfinding()) {
        auto optimal_path = env.getOptimalPath();
        std::string path_info = "Path: " + std::to_string(optimal_path.size()) + " waypoints";
        drawText(display_buffer_, path_info, cv::Point(20, y_offset), cv::Scalar(0, 255, 255), 0.6);
        y_offset += line_height;
        
        int current_waypoint = env.getCurrentWaypointIndex();
        std::string waypoint_info = "Waypoint: " + std::to_string(current_waypoint + 1) + "/" + std::to_string(optimal_path.size());
        drawText(display_buffer_, waypoint_info, cv::Point(20, y_offset), cv::Scalar(0, 255, 255), 0.6);
        y_offset += line_height;
    }
    
    // Agent info
    std::string agent_info = "Agent: " + env.getPathfindingAlgorithm();
    drawText(display_buffer_, agent_info, cv::Point(20, y_offset), cv::Scalar(255, 255, 255), 0.6);
    y_offset += line_height;
    
    // Mode info
    std::string mode_info = "Mode: " + getModeString();
    drawText(display_buffer_, mode_info, cv::Point(20, y_offset), cv::Scalar(255, 255, 255), 0.6);
}

std::string SimulatorUI::getModeString() const {
    switch (current_mode_) {
        case UIMode::SIMULATION: return "SIMULATION";
        case UIMode::MANUAL_PILOT: return "MANUAL";
        case UIMode::PAUSED: return "PAUSED";
        case UIMode::RECORDING: return "RECORDING";
        default: return "UNKNOWN";
    }
}

void SimulatorUI::renderCollisionBoxes(const sim::World& world) {
    // Get drone position for collision visualization
    // This is a simplified collision box rendering
    // In a full implementation, you'd get the actual collision boundaries
    
    // Draw safety margin around drone (if available)
    // For now, we'll just show a general safety zone
}

void SimulatorUI::renderRewardInfo(float reward) {
    // Create reward info panel (top-right corner)
    cv::Point panel_start(display_buffer_.cols - 200, 10);
    cv::Point panel_end(display_buffer_.cols - 10, 80);
    
    // Draw semi-transparent background
    cv::Mat overlay = display_buffer_.clone();
    cv::rectangle(overlay, panel_start, panel_end, cv::Scalar(0, 0, 0), -1);
    cv::addWeighted(display_buffer_, 0.7, overlay, 0.3, 0, display_buffer_);
    
    // Draw panel border
    cv::Scalar border_color = (reward > 0) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    cv::rectangle(display_buffer_, panel_start, panel_end, border_color, 2);
    
    // Render reward information
    cv::Point text_pos(panel_start.x + 10, panel_start.y + 25);
    drawText(display_buffer_, "REWARD", text_pos, cv::Scalar(255, 255, 255), 0.7);
    
    std::string reward_text = std::to_string(static_cast<int>(reward));
    cv::Point value_pos(panel_start.x + 10, panel_start.y + 50);
    drawText(display_buffer_, reward_text, value_pos, border_color, 0.8);
}

void SimulatorUI::renderFPS() {
    // Display FPS in top-right corner
    std::string fps_text = "FPS: " + std::to_string(static_cast<int>(current_fps_));
    cv::Point fps_pos(display_buffer_.cols - 120, 30);
    
    // Color code FPS
    cv::Scalar fps_color;
    if (current_fps_ > 50) fps_color = cv::Scalar(0, 255, 0);      // Green for good
    else if (current_fps_ > 30) fps_color = cv::Scalar(0, 255, 255); // Yellow for acceptable
    else fps_color = cv::Scalar(0, 0, 255);                          // Red for poor
    
    drawText(display_buffer_, fps_text, fps_pos, fps_color, 0.6);
}

cv::Point2f SimulatorUI::screenToWorld(const cv::Point2f& screen_pos) const {
    // Convert screen coordinates to world coordinates
    // This is a simplified conversion - in a real implementation you'd have proper scaling
    return screen_pos;
}

cv::Point2f SimulatorUI::worldToScreen(const cv::Point2f& world_pos) const {
    // Convert world coordinates to screen coordinates
    // This is a simplified conversion - in a real implementation you'd have proper scaling
    return world_pos;
}

void SimulatorUI::drawArrow(cv::Mat& img, const cv::Point2f& start, const cv::Point2f& end, 
                            const cv::Scalar& color, int thickness) {
    // Draw the main line
    cv::line(img, start, end, color, thickness);
    
    // Calculate arrow head
    float angle = atan2(end.y - start.y, end.x - start.x);
    float arrow_length = 15.0f;
    
    cv::Point2f arrow_head1 = end - cv::Point2f(arrow_length * cos(angle - M_PI/6), 
                                                 arrow_length * sin(angle - M_PI/6));
    cv::Point2f arrow_head2 = end - cv::Point2f(arrow_length * cos(angle + M_PI/6), 
                                                 arrow_length * sin(angle + M_PI/6));
    
    // Draw arrow head
    cv::line(img, end, arrow_head1, color, thickness);
    cv::line(img, end, arrow_head2, color, thickness);
}

void SimulatorUI::drawText(cv::Mat& img, const std::string& text, const cv::Point& pos, 
                           const cv::Scalar& color, double scale) {
    cv::putText(img, text, pos, cv::FONT_HERSHEY_SIMPLEX, scale, color, 2);
}

void SimulatorUI::handlePauseKey() {
    if (current_mode_ == UIMode::PAUSED) {
        current_mode_ = UIMode::SIMULATION;
        std::cout << "Simulation resumed" << std::endl;
    } else {
        current_mode_ = UIMode::PAUSED;
        std::cout << "Simulation paused" << std::endl;
    }
}

void SimulatorUI::handleManualModeKey() {
    if (current_mode_ == UIMode::MANUAL_PILOT) {
        current_mode_ = UIMode::SIMULATION;
        std::cout << "Switched to simulation mode" << std::endl;
    } else {
        current_mode_ = UIMode::MANUAL_PILOT;
        std::cout << "Switched to manual pilot mode" << std::endl;
    }
}

void SimulatorUI::handleRecordKey() {
    if (current_mode_ == UIMode::RECORDING) {
        stopRecording();
    } else {
        startRecording("recording.mp4", 30);
    }
}

void SimulatorUI::handleNewMapKey() {
    std::cout << "New map requested" << std::endl;
    // This will be handled by the main simulator
}

void SimulatorUI::handleResetKey() {
    std::cout << "Reset requested" << std::endl;
    // This will be handled by the main simulator
}

// Add method to get the display buffer for external use
cv::Mat SimulatorUI::getDisplayBuffer() const {
    return display_buffer_.clone();
}

// Add method to clear path trace
void SimulatorUI::clearPathTrace() {
    path_trace_.clear();
}

} // namespace ui
