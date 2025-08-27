#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include "sim/World.h"
#include "ui/SimulatorUI.h"

int main() {
    std::cout << "=== Debug SimulatorUI render3D ===" << std::endl;
    
    // Create a simple 3D world
    auto world = std::make_shared<sim::World>(800, 600, 400);
    world->generateMap(sim::MapType::SKYSCRAPER);
    
    // Create UI
    auto ui = std::make_shared<ui::SimulatorUI>();
    ui->createWindow("Debug UI", 1200, 800);
    
    // Set display options
    ui->setShowGrid(true);
    ui->setShowAxes(true);
    ui->setShowPath(true);
    ui->setShowObstacles(true);
    ui->setShowDrone(true);
    ui->setShowGoal(true);
    
    // Set 3D render mode
    ui->setRenderMode("3D");
    
    // Create a simple drone
    auto drone = std::make_shared<sim::Drone>(cv::Point3f(50.0f, 300.0f, 200.0f));
    
    // Create a simple path
    std::vector<cv::Point3f> path_3d = {
        cv::Point3f(50, 300, 200),
        cv::Point3f(200, 300, 200),
        cv::Point3f(400, 300, 200),
        cv::Point3f(600, 300, 200),
        cv::Point3f(750, 300, 200)
    };
    
    std::cout << "Testing SimulatorUI render3D..." << std::endl;
    
    // Test the render3D method
    ui->render3D(*world, *drone, path_3d, {});
    
    // Get the current frame
    cv::Mat current_frame = ui->getCurrentFrame();
    
    // Check the frame
    std::cout << "Current frame size: " << current_frame.cols << "x" << current_frame.rows << std::endl;
    std::cout << "Current frame type: " << current_frame.type() << std::endl;
    
    // Check some pixel values
    cv::Vec3b center_pixel = current_frame.at<cv::Vec3b>(400, 600);
    std::cout << "Center pixel (600, 400): B=" << (int)center_pixel[0] << " G=" << (int)center_pixel[1] << " R=" << (int)center_pixel[2] << std::endl;
    
    cv::Vec3b top_left_pixel = current_frame.at<cv::Vec3b>(0, 0);
    std::cout << "Top-left pixel (0, 0): B=" << (int)top_left_pixel[0] << " G=" << (int)top_left_pixel[1] << " R=" << (int)top_left_pixel[2] << std::endl;
    
    // Save the frame
    cv::imwrite("simulator_ui_debug.png", current_frame);
    std::cout << "Saved simulator_ui_debug.png" << std::endl;
    
    // Test updating the window
    std::cout << "Testing window update..." << std::endl;
    ui->updateWindow();
    
    // Wait a bit to see the window
    std::cout << "Window should be visible now. Press any key to continue..." << std::endl;
    cv::waitKey(3000);
    
    // Close the window
    ui->closeWindow();
    
    std::cout << "\n=== Debug SimulatorUI completed ===" << std::endl;
    
    return 0;
}
