#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include "sim/World.h"

int main() {
    std::cout << "=== Debug 3D Rendering ===" << std::endl;
    
    // Create a simple 3D world
    auto world = std::make_shared<sim::World>(800, 600, 400);
    
    // Generate skyscraper map
    std::cout << "Generating skyscraper map..." << std::endl;
    world->generateMap(sim::MapType::SKYSCRAPER);
    
    // Check obstacles
    const auto& obstacles = world->getObstacles();
    std::cout << "Generated " << obstacles.size() << " obstacles" << std::endl;
    
    for (size_t i = 0; i < std::min(size_t(5), obstacles.size()); ++i) {
        const auto& obs = obstacles[i];
        std::cout << "Obstacle " << i << ": pos=(" << obs.position.x << ", " << obs.position.y << ", " << obs.position.z 
                  << "), radius=" << obs.radius << ", height=" << obs.height << ", vertical=" << obs.is_vertical << std::endl;
    }
    
    // Test 3D rendering with different camera positions
    cv::Mat render_output;
    
    // Test 1: Camera above looking down
    std::cout << "\nTest 1: Camera above looking down" << std::endl;
    cv::Point3f camera_pos(400, 300, 600);
    cv::Point3f camera_target(400, 300, 200);
    world->render3D(render_output, camera_pos, camera_target);
    cv::imwrite("debug_render_above.png", render_output);
    std::cout << "Saved debug_render_above.png" << std::endl;
    
    // Test 2: Camera at ground level looking forward
    std::cout << "\nTest 2: Camera at ground level looking forward" << std::endl;
    camera_pos = cv::Point3f(50, 300, 200);
    camera_target = cv::Point3f(750, 300, 200);
    world->render3D(render_output, camera_pos, camera_target);
    cv::imwrite("debug_render_ground.png", render_output);
    std::cout << "Saved debug_render_ground.png" << std::endl;
    
    // Test 3: Camera from side
    std::cout << "\nTest 3: Camera from side" << std::endl;
    camera_pos = cv::Point3f(800, 100, 300);
    camera_target = cv::Point3f(400, 300, 200);
    world->render3D(render_output, camera_pos, camera_target);
    cv::imwrite("debug_render_side.png", render_output);
    std::cout << "Saved debug_render_side.png" << std::endl;
    
    // Test 4: Simple test with just a few obstacles
    std::cout << "\nTest 4: Simple test with minimal obstacles" << std::endl;
    auto simple_world = std::make_shared<sim::World>(800, 600, 400);
    
    // Add just one obstacle in the center
    sim::Obstacle test_obs;
    test_obs.position = cv::Point3f(400, 300, 200);
    test_obs.radius = 50.0f;
    test_obs.height = 100.0f;
    test_obs.is_vertical = true;
    test_obs.is_moving = false;
    test_obs.velocity = cv::Point3f(0, 0, 0);
    test_obs.max_speed = 0.0f;
    
    // We can't directly access private obstacles, so let's use the public interface
    // For now, let's just test with the existing world
    
    camera_pos = cv::Point3f(400, 300, 500);
    camera_target = cv::Point3f(400, 300, 200);
    world->render3D(render_output, camera_pos, camera_target);
    cv::imwrite("debug_render_simple.png", render_output);
    std::cout << "Saved debug_render_simple.png" << std::endl;
    
    std::cout << "\n=== Debug 3D Rendering Completed ===" << std::endl;
    std::cout << "Check the generated PNG files to see what's happening." << std::endl;
    
    return 0;
}
