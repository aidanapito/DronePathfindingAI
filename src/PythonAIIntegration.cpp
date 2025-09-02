#include "PythonAIIntegration.h"
#include <filesystem>
#include <sstream>
#include <iomanip>

PythonAIIntegration::PythonAIIntegration(const std::string& data_dir) 
    : data_dir_(data_dir), python_ai_enabled_(false), current_mode_(AIMode::MANUAL) {
    
    // Create data directory if it doesn't exist
    std::filesystem::create_directories(data_dir_);
    
    // Set up file paths
    drone_state_file_ = data_dir_ + "/drone_state.json";
    obstacles_file_ = data_dir_ + "/obstacles.json";
    ai_input_file_ = data_dir_ + "/ai_input.json";
    command_file_ = data_dir_ + "/command.json";
    
    std::cout << "🤖 Python AI Integration initialized" << std::endl;
    std::cout << "📁 Data directory: " << data_dir_ << std::endl;
}

PythonAIIntegration::~PythonAIIntegration() {
    // Clean up any temporary files if needed
}

void PythonAIIntegration::writeDroneState(const DroneState& drone_state) {
    if (!python_ai_enabled_) return;
    
    std::string json_data = droneStateToJSON(drone_state);
    writeJSON(drone_state_file_, json_data);
}

void PythonAIIntegration::writeObstacles(const std::vector<Obstacle>& obstacles) {
    if (!python_ai_enabled_) return;
    
    std::string json_data = obstaclesToJSON(obstacles);
    writeJSON(obstacles_file_, json_data);
}

void PythonAIIntegration::writeCommand(AIMode mode, const glm::vec3& target) {
    std::cout << "🔧 writeCommand called - python_ai_enabled_: " << (python_ai_enabled_ ? "true" : "false") << std::endl;
    
    if (!python_ai_enabled_) {
        std::cout << "❌ Python AI not enabled, skipping writeCommand" << std::endl;
        return;
    }
    
    current_mode_ = mode;
    std::string json_data = commandToJSON(mode, target);
    std::cout << "📝 Writing command.json to: " << command_file_ << std::endl;
    writeJSON(command_file_, json_data);
    std::cout << "📝 Writing command.json: " << json_data << std::endl;
}

DroneInput PythonAIIntegration::readAIInput() {
    if (!python_ai_enabled_ || !hasAIInput()) {
        return DroneInput{0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    }
    
    std::string json_data = readJSON(ai_input_file_);
    DroneInput input = parseAIInput(json_data);
    
    // Remove the file after reading to prevent stale data
    std::filesystem::remove(ai_input_file_);
    
    return input;
}

bool PythonAIIntegration::hasAIInput() const {
    return std::filesystem::exists(ai_input_file_);
}

void PythonAIIntegration::writeJSON(const std::string& filename, const std::string& json_data) {
    try {
        std::cout << "🔧 writeJSON called for: " << filename << std::endl;
        std::ofstream file(filename);
        if (file.is_open()) {
            file << json_data;
            file.close();
            std::cout << "✅ Successfully wrote to: " << filename << std::endl;
        } else {
            std::cerr << "❌ Failed to open file for writing: " << filename << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "❌ Error writing JSON file: " << e.what() << std::endl;
    }
}

std::string PythonAIIntegration::readJSON(const std::string& filename) {
    try {
        std::ifstream file(filename);
        if (file.is_open()) {
            std::stringstream buffer;
            buffer << file.rdbuf();
            file.close();
            return buffer.str();
        }
    } catch (const std::exception& e) {
        std::cerr << "❌ Error reading JSON file: " << e.what() << std::endl;
    }
    return "";
}

std::string PythonAIIntegration::droneStateToJSON(const DroneState& drone_state) {
    std::ostringstream json;
    json << std::fixed << std::setprecision(6);
    json << "{\n";
    json << "  \"x\": " << drone_state.x << ",\n";
    json << "  \"y\": " << drone_state.y << ",\n";
    json << "  \"z\": " << drone_state.z << ",\n";
    json << "  \"vx\": " << drone_state.vx << ",\n";
    json << "  \"vy\": " << drone_state.vy << ",\n";
    json << "  \"vz\": " << drone_state.vz << ",\n";
    json << "  \"roll\": " << drone_state.roll << ",\n";
    json << "  \"pitch\": " << drone_state.pitch << ",\n";
    json << "  \"yaw\": " << drone_state.yaw << "\n";
    json << "}";
    return json.str();
}

std::string PythonAIIntegration::obstaclesToJSON(const std::vector<Obstacle>& obstacles) {
    std::ostringstream json;
    json << std::fixed << std::setprecision(6);
    json << "[\n";
    
    for (size_t i = 0; i < obstacles.size(); ++i) {
        const auto& obs = obstacles[i];
        json << "  {\n";
        json << "  \"x\": " << obs.x << ",\n";
        json << "  \"y\": " << obs.y << ",\n";
        json << "  \"z\": " << obs.z << ",\n";
        json << "  \"width\": " << obs.width << ",\n";
        json << "  \"height\": " << obs.height << ",\n";
        json << "  \"radius\": " << obs.radius << "\n";
        json << "  }";
        if (i < obstacles.size() - 1) json << ",";
        json << "\n";
    }
    
    json << "]";
    return json.str();
}

std::string PythonAIIntegration::commandToJSON(AIMode mode, const glm::vec3& target) {
    std::ostringstream json;
    json << std::fixed << std::setprecision(6);
    json << "{\n";
    json << "  \"mode\": " << static_cast<int>(mode) << ",\n";
    json << "  \"target\": {\n";
    json << "  \"x\": " << target.x << ",\n";
    json << "  \"y\": " << target.y << ",\n";
    json << "  \"z\": " << target.z << "\n";
    json << "  }\n";
    json << "}";
    return json.str();
}

DroneInput PythonAIIntegration::parseAIInput(const std::string& json_data) {
    DroneInput input{0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    
    // More robust JSON parsing
    try {
        // Find and parse forward_thrust
        size_t pos = json_data.find("\"forward_thrust\":");
        if (pos != std::string::npos) {
            pos += 16; // Skip "forward_thrust":
            size_t end = json_data.find_first_of(",}", pos);
            if (end != std::string::npos) {
                std::string value_str = json_data.substr(pos, end - pos);
                input.forward_thrust = std::stof(value_str);
            }
        }
        
        // Find and parse yaw_rate
        pos = json_data.find("\"yaw_rate\":");
        if (pos != std::string::npos) {
            pos += 11; // Skip "yaw_rate":
            size_t end = json_data.find_first_of(",}", pos);
            if (end != std::string::npos) {
                std::string value_str = json_data.substr(pos, end - pos);
                input.yaw_rate = std::stof(value_str);
            }
        }
        
        // Find and parse pitch_rate
        pos = json_data.find("\"pitch_rate\":");
        if (pos != std::string::npos) {
            pos += 13; // Skip "pitch_rate":
            size_t end = json_data.find_first_of(",}", pos);
            if (end != std::string::npos) {
                std::string value_str = json_data.substr(pos, end - pos);
                input.pitch_rate = std::stof(value_str);
            }
        }
        
        // Find and parse roll_rate
        pos = json_data.find("\"roll_rate\":");
        if (pos != std::string::npos) {
            pos += 12; // Skip "roll_rate":
            size_t end = json_data.find_first_of(",}", pos);
            if (end != std::string::npos) {
                std::string value_str = json_data.substr(pos, end - pos);
                input.roll_rate = std::stof(value_str);
            }
        }
        
        // Find and parse vertical_thrust
        pos = json_data.find("\"vertical_thrust\":");
        if (pos != std::string::npos) {
            pos += 18; // Skip "vertical_thrust":
            size_t end = json_data.find_first_of(",}", pos);
            if (end != std::string::npos) {
                std::string value_str = json_data.substr(pos, end - pos);
                input.vertical_thrust = std::stof(value_str);
            }
        }
        
        std::cout << "✅ Parsed AI input: F:" << input.forward_thrust 
                  << " Y:" << input.yaw_rate 
                  << " V:" << input.vertical_thrust << std::endl;
                  
    } catch (const std::exception& e) {
        std::cerr << "❌ Error parsing AI input: " << e.what() << std::endl;
        std::cerr << "JSON data: " << json_data << std::endl;
    }
    
    return input;
}
