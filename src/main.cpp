#include "Drone.h"
#include "World.h"
#include "Camera.h"
#include "InputHandler.h"
#include "Renderer.h"
#include <iostream>
#include <map>
#include <chrono>

// Global variables for input handling
std::map<int, bool> key_pressed;
InputHandler* global_input_handler = nullptr;

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
    
    // Set global input handler for callbacks
    global_input_handler = &input;
    
    // Set camera reference in input handler for immediate mode switching
    input.setCamera(&camera);
    
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
        
        // Update drone physics
        DroneInput drone_input = input.getCurrentInput();
        drone.update(delta_time, drone_input);
        
        // Get current drone state
        DroneState drone_pos = drone.getPosition();
        DroneState drone_orient = drone.getOrientationOnly();  // Get only orientation values
        
        // Handle camera mode switching if requested
        if (input.shouldSwitchCameraMode()) {
            camera.switchMode(drone_pos, drone_orient);
            input.resetCameraSwitchFlag(); // Reset the flag after handling
        }
        
        // Update camera with current drone state
        camera.update(drone_pos, drone_orient);
        
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
        
        glm::vec3 gl_camera_pos(camera_pos.x, camera_pos.y, camera_pos.z);
        glm::vec3 gl_camera_target(camera_target.x, camera_target.y, camera_target.z);
        glm::vec3 gl_camera_up(camera_up.x, camera_up.y, camera_up.z);
        
        renderer.render3DScene(gl_camera_pos, gl_camera_target, gl_camera_up, world.getObstacles());
        
        // Render the drone with H-frame design
        glm::vec3 drone_position(drone_pos.x, drone_pos.y, drone_pos.z);
        glm::vec3 drone_orientation(drone_orient.roll, drone_orient.pitch, drone_orient.yaw);
        glm::vec3 drone_color(0.0f, 0.7f, 1.0f); // Bright blue color
        
        renderer.renderHFrameDrone(drone_position, drone_orientation, drone_color);
        
        // End frame and swap buffers
        renderer.endFrame();
        renderer.swapBuffers();
        
        // Poll events
        renderer.pollEvents();
        
        // Debug output
        std::cout << "\rDrone: pos(" << drone_pos.x << ", " << drone_pos.y << ", " << drone_pos.z 
                  << ") roll:" << drone_orient.roll << " pitch:" << drone_orient.pitch << " yaw:" << drone_orient.yaw 
                  << " | Input - T:" << drone_input.forward_thrust << " Y:" << drone_input.yaw_rate << " P:" << drone_input.pitch_rate 
                  << " R:" << drone_input.roll_rate << " V:" << drone_input.vertical_thrust << "    " << std::flush;
    }
    
    std::cout << "\n👋 Simulation ended. Goodbye!" << std::endl;
    return 0;
}
