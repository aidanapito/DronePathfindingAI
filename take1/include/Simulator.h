#pragma once

#include "sim/World.h"
#include "sim/Drone.h"
#include "agent/Agent.h"
#include "bridge/Environment.h"
#include "ui/SimulatorUI.h"
#include <memory>
#include <chrono>

class Simulator {
public:
    Simulator();
    ~Simulator() = default;

    // Main simulation loop
    void run();
    void step();
    void reset();
    
    // Configuration
    void setAgent(std::shared_ptr<agent::Agent> agent);
    void setMapType(sim::MapType map_type);
    void setLearningTrack(bool use_vision);
    
    // Control
    void pause();
    void resume();
    void togglePause();
    bool isPaused() const { return paused_; }
    
    // Statistics
    int getEpisodeCount() const { return episode_count_; }
    float getAverageReward() const;
    float getSuccessRate() const;

private:
    // Components
    std::shared_ptr<sim::World> world_;
    std::shared_ptr<sim::Drone> drone_;
    std::shared_ptr<agent::Agent> agent_;
    std::shared_ptr<bridge::Environment> environment_;
    std::shared_ptr<ui::SimulatorUI> ui_;
    
    // Simulation state
    bool paused_;
    bool running_;
    int episode_count_;
    
    // Timing
    std::chrono::high_resolution_clock::time_point last_frame_time_;
    float target_fps_;
    float time_step_;
    
    // Configuration
    sim::MapType current_map_type_;
    bool use_vision_track_;
    
    // Helper methods
    void initializeComponents();
    void setupEpisode();
    void handleUserInput();
    void updateSimulation();
    void renderFrame();
    void recordFrame();
    std::string getMapTypeName(sim::MapType map_type);
    
    // Agent selection
    std::shared_ptr<agent::Agent> createAgent();
    void configureEnvironment();
};

// Global configuration
struct SimulatorConfig {
    int window_width = 1200;
    int window_height = 800;
    float target_fps = 60.0f;
    float time_step = 1.0f / 60.0f;
    int max_episodes = 1000;
    bool enable_video_recording = false;
    std::string video_output_dir = "recordings/";
    
    // Learning parameters
    bool use_vision_track = false;
    float learning_rate = 0.001f;
    float discount_factor = 0.99f;
    float epsilon = 0.1f;
    
    // World parameters
    sim::MapType default_map_type = sim::MapType::MAZE;
    int world_width = 800;
    int world_height = 600;
    int obstacle_count = 20;
    int moving_obstacle_count = 5;
};
