#include "Drone.h"
#include "World.h"
#include "Camera.h"
#include "InputHandler.h"
#include "Renderer.h"
#include "AI/PathfindingAI.h"
#include <iostream>
#include <map>
#include <chrono>
#include <cmath> // For sqrt

// Global variables for input handling
std::map<int, bool> key_pressed;
InputHandler* global_input_handler = nullptr;

// Game state
enum class GameState {
    PLAYING,
    CRASHED
};

GameState current_game_state = GameState::PLAYING;

// Mouse callback for OpenGL
void onMouse(GLFWwindow* window, double xpos, double ypos) {
    if (global_input_handler) {
        global_input_handler->processMouse(xpos, ypos);
    }
}

// Key callback for OpenGL
void onKey(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (global_input_handler) {
        if (action == GLFW_PRESS) {
            global_input_handler->processKey(key, true);
            key_pressed[key] = true;
        } else if (action == GLFW_RELEASE) {
            global_input_handler->processKey(key, false);
            key_pressed[key] = false;
        }
    }
}

int main() {
    std::cout << "🚁 3D Drone Simulator with OpenGL" << std::endl;
    std::cout << "Initializing..." << std::endl;
    
    // Initialize OpenGL renderer
    Renderer renderer(1200, 800);
    if (!renderer.initialize()) {
        std::cerr << "Failed to initialize OpenGL renderer!" << std::endl;
        return -1;
    }
    
    // Initialize components
    World world(1200, 800, 600);
    Drone drone;
    Camera camera;
    InputHandler input;
    PathfindingAI ai_controller;
    
    // Set global input handler for callbacks
    global_input_handler = &input;
    
    // Set camera reference in input handler for immediate mode switching
    input.setCamera(&camera);
    
    // Set up AI controller
    input.setAIController(&ai_controller);
    ai_controller.setHomePosition(0.0f, 0.0f, 50.0f);
    
    // Set target to the target building location
    const Obstacle* target_building = world.getTargetBuilding();
    if (target_building) {
        // Target is the top of the building
        float target_x = target_building->x;
        float target_y = target_building->y;
        float target_z = target_building->z + target_building->height; // Top of building
        ai_controller.setTarget(target_x, target_y, target_z);
        std::cout << "🎯 AI target set to building top: (" << target_x << ", " << target_y << ", " << target_z << ")" << std::endl;
    }
    
    // Set up OpenGL callbacks
    GLFWwindow* window = renderer.getWindow();
    glfwSetCursorPosCallback(window, onMouse);
    glfwSetKeyCallback(window, onKey);
    
    // Generate world
    world.generateTerrain();
    
    // Set initial camera mode
    camera.setFirstPersonMode(drone.getPosition(), drone.getOrientation());
    
    std::cout << "✅ Initialization complete!" << std::endl;
    std::cout << "🎮 Controls:" << std::endl;
    std::cout << "   W/S: Forward/Backward" << std::endl;
    std::cout << "   A/D: Roll Left/Right" << std::endl;
    std::cout << "   Q/E: Yaw Left/Right" << std::endl;
    std::cout << "   R/F: Pitch Up/Down" << std::endl;
    std::cout << "   Z/X: Fly Up/Down" << std::endl;
    std::cout << "   Mouse: Look around (1st person) / Orbit (3rd person)" << std::endl;
    std::cout << "   Space: Switch camera mode" << std::endl;
    std::cout << "   ESC: Exit" << std::endl;
    std::cout << "🤖 AI Controls:" << std::endl;
    std::cout << "   T: Toggle AI Control" << std::endl;
    std::cout << "   1-5: AI Modes (Manual/Follow/Explore/Return/Avoid)" << std::endl;
    std::cout << "   Note: Press 'T' first to enable AI, then press mode number" << std::endl;
    std::cout << "   Mode 2 (Follow Path): Drone will plan and follow a path to the target building" << std::endl;
    
    // Game loop
    auto last_time = std::chrono::high_resolution_clock::now();
    
    while (!renderer.shouldClose()) {
        auto current_time = std::chrono::high_resolution_clock::now();
        float delta_time = std::chrono::duration<float>(current_time - last_time).count();
        last_time = current_time;
        
        // Process input
        for (const auto& [key, pressed] : key_pressed) {
            input.processKey(key, pressed);
        }
        
        // Update input handler to calculate current input values
        input.update(delta_time);
        
        // Get drone input (manual or AI)
        DroneInput drone_input;
        if (input.isAIControlEnabled()) {
            // Use AI control
            DroneState current_state = drone.getPosition();
            
            // Update path planning for FOLLOW_PATH mode
            if (ai_controller.getMode() == AIMode::FOLLOW_PATH && 
                (ai_controller.getState() == AIState::IDLE || ai_controller.getState() == AIState::PLANNING_PATH)) {
                ai_controller.updatePath(current_state, world);
            }
            
            drone_input = ai_controller.update(delta_time, current_state, world);
        } else {
            // Use manual control
            drone_input = input.getCurrentInput();
        }
        
        // Update drone physics
        drone.update(delta_time, drone_input);
        
        // Get current drone state
        DroneState drone_pos = drone.getPosition();
        DroneState drone_orient = drone.getOrientation();  // Get complete state with position and orientation
        
        // Check for collisions if playing
        if (current_game_state == GameState::PLAYING) {
            // Debug: Print drone position occasionally
            static int debug_counter = 0;
            if (++debug_counter % 60 == 0) { // Every 60 frames (about once per second)
                std::cout << "\rDrone at: (" << drone_pos.x << ", " << drone_pos.y << ", " << drone_pos.z << ") - ";
                // Check if we're near any buildings
                bool near_building = false;
                for (const auto& obs : world.getObstacles()) {
                    float dx = drone_pos.x - obs.x;
                    float dy = drone_pos.y - obs.y;
                    float dz = drone_pos.z - obs.z;
                    float distance = sqrt(dx*dx + dy*dy + dz*dz);
                    if (distance < 50.0f) { // Within 50 units of a building
                        near_building = true;
                        break;
                    }
                }
                std::cout << (near_building ? "Near building" : "Not near building") << "    " << std::flush;
            }
            
            // Check collision with buildings (using smaller drone radius for more sensitive detection)
            if (world.checkCollision(drone_pos.x, drone_pos.y, drone_pos.z, 2.0f)) {
                current_game_state = GameState::CRASHED;
                std::cout << "\n💥 CRASH! You hit a building!" << std::endl;
                std::cout << "Drone position: (" << drone_pos.x << ", " << drone_pos.y << ", " << drone_pos.z << ")" << std::endl;
                std::cout << "Press 'R' to restart or 'ESC' to exit" << std::endl;
                std::cout << "Game state changed to CRASHED" << std::endl;
            }
        }
        
        // Handle restart if crashed
        if (current_game_state == GameState::CRASHED) {
            if (key_pressed[GLFW_KEY_R]) {
                // Reset game state
                current_game_state = GameState::PLAYING;
                
                // Reset drone to starting position
                drone = Drone(); // Create new drone with default position
                
                // Reset camera
                camera.setFirstPersonMode(drone.getPosition(), drone.getOrientation());
                
                // Reset renderer state properly
                renderer.clear(glm::vec3(0.5f, 0.7f, 1.0f)); // Reset to sky blue
                
                // Force a complete screen refresh with proper projection
                renderer.beginFrame();
                renderer.clear(glm::vec3(0.5f, 0.7f, 1.0f));
                
                // Clear any held keys to prevent immediate restart
                key_pressed.clear();
                
                std::cout << "Game restarted!" << std::endl;
            }
        }
        
        // Handle camera mode switching if requested (only if playing)
        if (current_game_state == GameState::PLAYING && input.shouldSwitchCameraMode()) {
            camera.switchMode(drone_pos, drone_orient);
            input.resetCameraSwitchFlag(); // Reset the flag after handling
        }
        
        // Update camera with current drone state (only if playing)
        if (current_game_state == GameState::PLAYING) {
            camera.update(drone_pos, drone_orient);
        }
        
        // Handle pause
        if (input.isPauseRequested()) {
            // Toggle pause state if needed
        }
        
        // Handle exit
        if (input.isExitRequested()) {
            break;
        }
        
        // Begin OpenGL frame
        renderer.beginFrame();
        renderer.clear(glm::vec3(0.5f, 0.7f, 1.0f)); // Sky blue
        
        // Render 3D scene
        CameraState camera_pos = camera.getPosition();
        CameraState camera_target = camera.getTarget();
        CameraState camera_up = camera.getUp();
        
        // Transform camera coordinates from our system (X=left/right, Y=forward/backward, Z=up/down)
        // to OpenGL system (X=left/right, Y=up/down, Z=forward/backward)
        glm::vec3 gl_camera_pos(camera_pos.x, camera_pos.z, camera_pos.y);
        glm::vec3 gl_camera_target(camera_target.x, camera_target.z, camera_target.y);
        glm::vec3 gl_camera_up(camera_up.x, camera_up.z, camera_up.y);
        
        renderer.render3DScene(gl_camera_pos, gl_camera_target, gl_camera_up, world.getObstacles(), world.getTargetBuilding());
        
        // Render the drone with H-frame design
        // Transform drone coordinates to match OpenGL system
        glm::vec3 drone_position(drone_pos.x, drone_pos.z, drone_pos.y);
        // Transform drone orientation to match OpenGL coordinate system
        // Roll stays the same, pitch and yaw need to be adjusted for coordinate system change
        glm::vec3 drone_orientation(drone_orient.roll, drone_orient.pitch, drone_orient.yaw);
        glm::vec3 drone_color(0.0f, 0.7f, 1.0f); // Bright blue color
        
        // Check if we're in third-person mode
        bool isThirdPerson = (camera.getMode() == CameraMode::THIRD_PERSON);
        renderer.renderXFrameDrone(drone_position, drone_orientation, drone_color, isThirdPerson);
        
        // Render crash message if crashed
        if (current_game_state == GameState::CRASHED) {
            std::cout << "\rRendering crash message..." << std::flush;
            renderer.renderCrashMessage();
        }
        
        // End frame and swap buffers
        renderer.endFrame();
        renderer.swapBuffers();
        
        // Poll events
        renderer.pollEvents();
        
        // Debug output
        std::cout << "\rDrone: pos(" << drone_pos.x << ", " << drone_pos.y << ", " << drone_pos.z 
                  << ") roll:" << drone_orient.roll << " pitch:" << drone_orient.pitch << " yaw:" << drone_orient.yaw 
                  << " | Input - T:" << drone_input.forward_thrust << " Y:" << drone_input.yaw_rate << " P:" << drone_input.pitch_rate 
                  << " R:" << drone_input.roll_rate << " V:" << drone_input.vertical_thrust;
        
        if (input.isAIControlEnabled()) {
            std::cout << " | AI:" << static_cast<int>(ai_controller.getMode()) << " State:" << static_cast<int>(ai_controller.getState());
            glm::vec3 target = ai_controller.getCurrentTarget();
            std::cout << " Target:(" << target.x << "," << target.y << "," << target.z << ")";
            
            // Show path information for FOLLOW_PATH mode
            if (ai_controller.getMode() == AIMode::FOLLOW_PATH) {
                const auto& path = ai_controller.getCurrentPath();
                if (!path.empty()) {
                    std::cout << " Path:" << path.size() << "pts";
                    if (ai_controller.getState() == AIState::FOLLOWING_PATH) {
                        std::cout << " WP:" << ai_controller.getCurrentWaypointIndex() << "/" << path.size();
                    }
                }
            }
            
            // Check if drone has landed on target
            if (ai_controller.hasLandedOnTarget(drone_pos, world)) {
                std::cout << " 🎯 LANDED ON TARGET!";
            }
        }
        
        std::cout << "    " << std::flush;
    }
    
    std::cout << "\n👋 Simulation ended. Goodbye!" << std::endl;
    return 0;
}
