#pragma once

#include <opencv2/opencv.hpp>
#include <memory>
#include <string>

namespace sim {
    class World;
    class Drone;
}

namespace agent {
    class Agent;
}

namespace bridge {
    class Environment;
}

namespace ui {

enum class UIMode {
    SIMULATION,      // Normal simulation mode
    MANUAL_PILOT,   // Manual control mode
    PAUSED,         // Paused simulation
    RECORDING       // Recording video
};

struct UIConfig {
    std::string window_name;
    int window_width;
    int window_height;
    bool show_debug_info;
    bool show_path_trace;
    bool show_collision_boxes;
    bool show_reward_info;
    bool show_fps;
    
    // Keybindings
    char pause_key;
    char manual_mode_key;
    char record_key;
    char new_map_key;
    char reset_key;
};

class SimulatorUI {
public:
    SimulatorUI(const UIConfig& config);
    ~SimulatorUI() = default;

    // Main rendering
    void render(const sim::World& world, const sim::Drone& drone, 
                const agent::Agent* agent = nullptr);
    void renderUI(const bridge::Environment& env);
    
    // Event handling
    bool handleKeyPress(int key);
    void handleMouseEvent(int event, int x, int y, int flags);
    
    // Mode management
    void setMode(UIMode mode);
    UIMode getMode() const { return current_mode_; }
    
    // Video recording
    void startRecording(const std::string& output_path, int fps = 30);
    void stopRecording();
    bool isRecording() const { return video_writer_.isOpened(); }
    
    // Configuration
    void setConfig(const UIConfig& config);
    const UIConfig& getConfig() const { return config_; }

private:
    UIConfig config_;
    UIMode current_mode_;
    
    // OpenCV components
    cv::VideoWriter video_writer_;
    cv::Mat display_buffer_;
    
    // UI state
    bool show_debug_overlay_;
    std::vector<cv::Point2f> path_trace_;
    float current_fps_;
    float last_frame_time_;
    
    // Helper methods
    void renderWorld(const sim::World& world);
    void renderDrone(const sim::Drone& drone);
    void renderPathTrace();
    void renderDebugInfo(const bridge::Environment& env);
    void renderCollisionBoxes(const sim::World& world);
    void renderRewardInfo(float reward);
    void renderFPS();
    
    // Utility
    cv::Point2f screenToWorld(const cv::Point2f& screen_pos) const;
    cv::Point2f worldToScreen(const cv::Point2f& world_pos) const;
    void drawArrow(cv::Mat& img, const cv::Point2f& start, const cv::Point2f& end, 
                   const cv::Scalar& color, int thickness = 2);
    void drawText(cv::Mat& img, const std::string& text, const cv::Point& pos, 
                  const cv::Scalar& color, double scale = 0.6);
    
    // Keybinding handlers
    void handlePauseKey();
    void handleManualModeKey();
    void handleRecordKey();
    void handleNewMapKey();
    void handleResetKey();
};

} // namespace ui
