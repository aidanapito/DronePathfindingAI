#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <map>
#include "World.h"
#include "Drone.h"
#include "Camera.h"
#include "InputHandler.h"

// Mouse callback for OpenCV window
void onMouse(int event, int x, int y, int flags, void* userdata) {
    if (userdata) {
        InputHandler* input = static_cast<InputHandler*>(userdata);
        input->processMouse(event, x, y, flags);
    }
}

int main() {
    std::cout << "🚁 3D Drone Simulation Starting..." << std::endl;
    
    // Create OpenCV window
    const std::string window_name = "3D Drone Simulator";
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
    
    // Initialize components
    World world(1200, 800, 600);
    Drone drone(cv::Point3f(100, 100, 150));
    Camera camera;
    InputHandler input;
    
    // Set mouse callback after input handler is created
    cv::setMouseCallback(window_name, onMouse, &input);
    
    // Generate world terrain
    world.generateTerrain();
    
    // Set initial camera to first-person
    camera.setFirstPersonMode(drone.getPosition(), drone.getOrientation());
    
    // Game loop variables
    auto last_time = std::chrono::high_resolution_clock::now();
    bool paused = false;
    
    // Input tracking
    std::map<int, bool> key_pressed;
    
    std::cout << "🎮 Controls:" << std::endl;
    std::cout << "   WASD - Move drone" << std::endl;
    std::cout << "   Q/E - Rotate left/right" << std::endl;
    std::cout << "   Space - Toggle pause" << std::endl;
    std::cout << "   C - Toggle camera (1st/3rd person)" << std::endl;
    std::cout << "   ESC - Exit" << std::endl;
    std::cout << "   Mouse drag - Orbit camera (3rd person)" << std::endl;
    std::cout << "   Mouse wheel - Zoom (3rd person)" << std::endl;
    
    while (!input.shouldExit()) {
        auto current_time = std::chrono::high_resolution_clock::now();
        float delta_time = std::chrono::duration<float>(current_time - last_time).count();
        last_time = current_time;
        
        // Cap delta time to prevent huge jumps
        delta_time = std::min(delta_time, 0.1f);
        
        // Handle input
        input.update();
        
        // Handle camera toggle
        if (input.shouldToggleCamera()) {
            if (camera.getMode() == CameraMode::FIRST_PERSON) {
                camera.setThirdPersonMode(drone.getPosition(), drone.getOrientation());
                std::cout << "📷 Switched to Third-Person Camera" << std::endl;
            } else {
                camera.setFirstPersonMode(drone.getPosition(), drone.getOrientation());
                std::cout << "📷 Switched to First-Person Camera" << std::endl;
            }
            input.clearCameraToggle();
        }
        
        // Handle pause toggle
        if (input.shouldPause()) {
            paused = !paused;
            std::cout << (paused ? "⏸️  Paused" : "▶️  Resumed") << std::endl;
            input.clearPauseToggle();
        }
        
        if (!paused) {
            // Update drone physics
            const DroneInput& drone_input = input.getDroneInput();
            drone.update(delta_time, 
                        drone_input.throttle, 
                        drone_input.yaw_rate,
                        drone_input.pitch_rate,
                        drone_input.roll_rate,
                        drone_input.vertical_thrust);
            
            // Update camera position
            if (camera.getMode() == CameraMode::FIRST_PERSON) {
                camera.setFirstPersonMode(drone.getPosition(), drone.getOrientation());
            } else {
                camera.setThirdPersonMode(drone.getPosition(), drone.getOrientation());
            }
        }
        
        // Render 3D world
        cv::Mat frame(800, 1200, CV_8UC3, cv::Scalar(135, 206, 235)); // Sky blue background
        
        // Debug: Print camera and drone info
        std::cout << "Camera: pos(" << camera.getPosition().x << ", " << camera.getPosition().y << ", " << camera.getPosition().z 
                  << ") target(" << camera.getTarget().x << ", " << camera.getTarget().y << ", " << camera.getTarget().z << ")" << std::endl;
        std::cout << "Drone: pos(" << drone.getPosition().x << ", " << drone.getPosition().y << ", " << drone.getPosition().z 
                  << ") heading: " << drone.getOrientation().z << std::endl;
        
        // Render world from camera perspective
        world.render3D(frame, camera.getPosition(), camera.getTarget());
        
        // Render drone
        cv::Point2f drone_screen = world.project3DTo2D(drone.getPosition(), 
                                                      camera.getPosition(), 
                                                      camera.getTarget());
        if (drone_screen.x >= 0 && drone_screen.x < frame.cols && 
            drone_screen.y >= 0 && drone_screen.y < frame.rows) {
            cv::circle(frame, drone_screen, 8, cv::Scalar(0, 255, 0), -1); // Green drone
        }
        
        // Render UI
        std::string mode_text = (camera.getMode() == CameraMode::FIRST_PERSON) ? "1st Person" : "3rd Person";
        cv::putText(frame, "Camera: " + mode_text, cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        
        cv::putText(frame, "Position: (" + std::to_string((int)drone.getPosition().x) + 
                   ", " + std::to_string((int)drone.getPosition().y) + 
                   ", " + std::to_string((int)drone.getPosition().z) + ")", 
                   cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, 
                   cv::Scalar(255, 255, 255), 2);
        
        if (paused) {
            cv::putText(frame, "PAUSED", cv::Point(frame.cols/2 - 100, frame.rows/2), 
                       cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(0, 0, 255), 3);
        }
        
        // Display frame
        cv::imshow(window_name, frame);
        
        // Handle key input
        int key = cv::waitKey(1) & 0xFF;
        if (key != 255) {
            if (!key_pressed[key]) {
                // Key was just pressed
                input.processKey(key, true);
                key_pressed[key] = true;
                std::cout << "🔑 Key pressed: " << (char)key << std::endl;
            }
        } else {
            // No key pressed, check for released keys
            for (auto& [k, pressed] : key_pressed) {
                if (pressed) {
                    input.processKey(k, false);
                    pressed = false;
                    std::cout << "🔑 Key released: " << (char)k << std::endl;
                }
            }
        }
        
        // Debug: Print current input values
        const DroneInput& current_input = input.getDroneInput();
        if (current_input.throttle != 0 || current_input.yaw_rate != 0 || 
            current_input.pitch_rate != 0 || current_input.roll_rate != 0) {
            std::cout << "🎮 Input: T=" << current_input.throttle 
                      << " Y=" << current_input.yaw_rate 
                      << " P=" << current_input.pitch_rate 
                      << " R=" << current_input.roll_rate << std::endl;
        }
        
        // Check for window close
        if (cv::getWindowProperty(window_name, cv::WND_PROP_VISIBLE) < 1) {
            break;
        }
    }
    
    std::cout << "👋 Simulation ended. Goodbye!" << std::endl;
    cv::destroyAllWindows();
    return 0;
}
