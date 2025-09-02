#ifndef PYTHON_AI_INTEGRATION_H
#define PYTHON_AI_INTEGRATION_H

#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include "AI/PathfindingAI.h"
#include "World.h"

class PythonAIIntegration {
private:
    std::string data_dir_;
    std::string drone_state_file_;
    std::string obstacles_file_;
    std::string ai_input_file_;
    std::string command_file_;
    
    bool python_ai_enabled_;
    AIMode current_mode_;
    
public:
    PythonAIIntegration(const std::string& data_dir = "ai_data");
    ~PythonAIIntegration();
    
    // Enable/disable Python AI
    void enablePythonAI(bool enable) { python_ai_enabled_ = enable; }
    bool isPythonAIEnabled() const { return python_ai_enabled_; }
    
    // Write drone state to JSON file
    void writeDroneState(const DroneState& drone_state);
    
    // Write obstacles to JSON file
    void writeObstacles(const std::vector<Obstacle>& obstacles);
    
    // Write command to JSON file
    void writeCommand(AIMode mode, const glm::vec3& target = glm::vec3(0, 0, 0));
    
    // Read AI input from JSON file
    DroneInput readAIInput();
    
    // Check if AI input file exists
    bool hasAIInput() const;
    
    // Get current mode
    AIMode getCurrentMode() const { return current_mode_; }
    
private:
    // Helper function to write JSON
    void writeJSON(const std::string& filename, const std::string& json_data);
    
    // Helper function to read JSON
    std::string readJSON(const std::string& filename);
    
    // Convert DroneState to JSON
    std::string droneStateToJSON(const DroneState& drone_state);
    
    // Convert obstacles to JSON
    std::string obstaclesToJSON(const std::vector<Obstacle>& obstacles);
    
    // Convert command to JSON
    std::string commandToJSON(AIMode mode, const glm::vec3& target);
    
    // Parse AI input from JSON
    DroneInput parseAIInput(const std::string& json_data);
};

#endif // PYTHON_AI_INTEGRATION_H
