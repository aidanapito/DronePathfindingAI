#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include "sim/World.h"

int main() {
    std::cout << "=== Detailed Debug 3D Rendering ===" << std::endl;
    
    // Create a simple 3D world
    auto world = std::make_shared<sim::World>(800, 600, 400);
    
    // Generate skyscraper map
    std::cout << "Generating skyscraper map..." << std::endl;
    world->generateMap(sim::MapType::SKYSCRAPER);
    
    // Check obstacles
    const auto& obstacles = world->getObstacles();
    std::cout << "Generated " << obstacles.size() << " obstacles" << std::endl;
    
    // Test camera positions
    cv::Point3f camera_pos(400, 300, 500);
    cv::Point3f camera_target(400, 300, 200);
    
    std::cout << "\nCamera position: (" << camera_pos.x << ", " << camera_pos.y << ", " << camera_pos.z << ")" << std::endl;
    std::cout << "Camera target: (" << camera_target.x << ", " << camera_target.y << ", " << camera_target.z << ")" << std::endl;
    std::cout << "Screen dimensions: " << 800 << "x" << 600 << std::endl;
    
    // Test projection calculations manually
    std::cout << "\nTesting projection calculations:" << std::endl;
    for (size_t i = 0; i < std::min(size_t(10), obstacles.size()); ++i) {
        const auto& obs = obstacles[i];
        
        // Calculate depth-based scaling
        float depth = obs.position.z - camera_pos.z;
        float scale = 1000.0f / (depth + 100.0f);
        float screen_x = (obs.position.x - camera_pos.x) * scale + 800 / 2;
        float screen_y = (obs.position.y - camera_pos.y) * scale + 600 / 2;
        float screen_radius = obs.radius * scale;
        
        std::cout << "Obstacle " << i << ": pos=(" << obs.position.x << ", " << obs.position.y << ", " << obs.position.z << ")" << std::endl;
        std::cout << "  Depth: " << depth << ", Scale: " << scale << std::endl;
        std::cout << "  Screen pos: (" << screen_x << ", " << screen_y << "), radius: " << screen_radius << std::endl;
        std::cout << "  In bounds: " << (screen_x >= 0 && screen_x < 800 && screen_y >= 0 && screen_y < 600 ? "YES" : "NO") << std::endl;
    }
    
    // Test start and goal positions
    cv::Point3f start_pos = world->getStartPosition();
    cv::Point3f goal_pos = world->getGoalPosition();
    
    std::cout << "\nStart position: (" << start_pos.x << ", " << start_pos.y << ", " << start_pos.z << ")" << std::endl;
    std::cout << "Goal position: (" << goal_pos.x << ", " << goal_pos.y << ", " << goal_pos.z << ")" << std::endl;
    
    // Test start/goal projection
    float start_scale = 1000.0f / (start_pos.z - camera_pos.z + 100.0f);
    float goal_scale = 1000.0f / (goal_pos.z - camera_pos.z + 100.0f);
    
    cv::Point2f start_screen((start_pos.x - camera_pos.x) * start_scale + 800 / 2,
                             (start_pos.y - camera_pos.y) * start_scale + 600 / 2);
    cv::Point2f goal_screen((goal_pos.x - camera_pos.x) * goal_scale + 800 / 2,
                            (goal_pos.y - camera_pos.y) * goal_scale + 600 / 2);
    
    std::cout << "Start screen: (" << start_screen.x << ", " << start_screen.y << "), scale: " << start_scale << std::endl;
    std::cout << "Goal screen: (" << goal_screen.x << ", " << goal_screen.y << "), scale: " << goal_scale << std::endl;
    
    // Now test the actual rendering
    std::cout << "\nTesting actual rendering..." << std::endl;
    cv::Mat render_output;
    world->render3D(render_output, camera_pos, camera_target);
    
    // Check the output image
    std::cout << "Render output size: " << render_output.cols << "x" << render_output.rows << std::endl;
    std::cout << "Render output type: " << render_output.type() << std::endl;
    
    // Check some pixel values
    cv::Vec3b center_pixel = render_output.at<cv::Vec3b>(300, 400);
    std::cout << "Center pixel (400, 300): B=" << (int)center_pixel[0] << " G=" << (int)center_pixel[1] << " R=" << (int)center_pixel[2] << std::endl;
    
    cv::Vec3b top_left_pixel = render_output.at<cv::Vec3b>(0, 0);
    std::cout << "Top-left pixel (0, 0): B=" << (int)top_left_pixel[0] << " G=" << (int)top_left_pixel[1] << " R=" << (int)top_left_pixel[2] << std::endl;
    
    // Save the render
    cv::imwrite("debug_render_detailed.png", render_output);
    std::cout << "Saved debug_render_detailed.png" << std::endl;
    
    // Create a simple test image to verify OpenCV is working
    cv::Mat test_image = cv::Mat::zeros(600, 800, CV_8UC3);
    cv::circle(test_image, cv::Point(400, 300), 100, cv::Scalar(0, 255, 0), -1);
    cv::circle(test_image, cv::Point(200, 200), 50, cv::Scalar(255, 0, 0), -1);
    cv::circle(test_image, cv::Point(600, 400), 75, cv::Scalar(0, 0, 255), -1);
    cv::imwrite("test_opencv.png", test_image);
    std::cout << "Saved test_opencv.png to verify OpenCV is working" << std::endl;
    
    std::cout << "\n=== Detailed Debug 3D Rendering Completed ===" << std::endl;
    
    return 0;
}
