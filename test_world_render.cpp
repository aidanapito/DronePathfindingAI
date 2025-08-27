#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include "sim/World.h"

int main() {
    std::cout << "=== Testing World render3D directly ===" << std::endl;
    
    // Create a simple 3D world
    auto world = std::make_shared<sim::World>(800, 600, 400);
    world->generateMap(sim::MapType::SKYSCRAPER);
    
    // Check obstacles
    const auto& obstacles = world->getObstacles();
    std::cout << "Generated " << obstacles.size() << " obstacles" << std::endl;
    
    // Test camera positions
    cv::Point3f camera_pos(400, 300, 500);
    cv::Point3f camera_target(400, 300, 200);
    
    std::cout << "Camera position: (" << camera_pos.x << ", " << camera_pos.y << ", " << camera_pos.z << ")" << std::endl;
    std::cout << "Camera target: (" << camera_target.x << ", " << camera_target.y << ", " << camera_target.z << ")" << std::endl;
    
    // Test the World's render3D method directly
    cv::Mat render_output;
    world->render3D(render_output, camera_pos, camera_target);
    
    // Check the output
    std::cout << "Render output size: " << render_output.cols << "x" << render_output.rows << std::endl;
    std::cout << "Render output type: " << render_output.type() << std::endl;
    
    // Check some pixel values
    cv::Vec3b center_pixel = render_output.at<cv::Vec3b>(300, 400);
    std::cout << "Center pixel (400, 300): B=" << (int)center_pixel[0] << " G=" << (int)center_pixel[1] << " R=" << (int)center_pixel[2] << std::endl;
    
    cv::Vec3b top_left_pixel = render_output.at<cv::Vec3b>(0, 0);
    std::cout << "Top-left pixel (0, 0): B=" << (int)top_left_pixel[0] << " G=" << (int)top_left_pixel[1] << " R=" << (int)top_left_pixel[2] << std::endl;
    
    // Save the render
    cv::imwrite("world_render_test.png", render_output);
    std::cout << "Saved world_render_test.png" << std::endl;
    
    // Now test with different camera position (looking from above)
    std::cout << "\nTesting with camera above..." << std::endl;
    camera_pos = cv::Point3f(400, 300, 600);
    camera_target = cv::Point3f(400, 300, 200);
    
    world->render3D(render_output, camera_pos, camera_target);
    cv::imwrite("world_render_above.png", render_output);
    std::cout << "Saved world_render_above.png" << std::endl;
    
    // Test with camera at ground level
    std::cout << "\nTesting with camera at ground level..." << std::endl;
    camera_pos = cv::Point3f(50, 300, 200);
    camera_target = cv::Point3f(750, 300, 200);
    
    world->render3D(render_output, camera_pos, camera_target);
    cv::imwrite("world_render_ground.png", render_output);
    std::cout << "Saved world_render_ground.png" << std::endl;
    
    std::cout << "\n=== World render3D test completed ===" << std::endl;
    
    return 0;
}
