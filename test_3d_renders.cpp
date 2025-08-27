#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include "sim/World.h"

int main() {
    std::cout << "=== 3D Rendering Test ===" << std::endl;
    
    // Test different 3D map types
    std::vector<sim::MapType> map_types = {
        sim::MapType::SKYSCRAPER,
        sim::MapType::UNDERWATER,
        sim::MapType::MOUNTAIN_PASS
    };
    
    for (auto map_type : map_types) {
        std::cout << "\n--- Generating " << (map_type == sim::MapType::SKYSCRAPER ? "Skyscraper" :
                                        map_type == sim::MapType::UNDERWATER ? "Underwater" : "Mountain Pass") 
                  << " Map Render ---" << std::endl;
        
        // Create 3D world
        auto world = std::make_shared<sim::World>(800, 600, 400);
        
        // Generate 3D map
        world->generateMap(map_type);
        
        // Test 3D rendering
        std::cout << "Generating 3D render..." << std::endl;
        cv::Mat render_output;
        cv::Point3f camera_pos(400, 300, 500);
        cv::Point3f camera_target(400, 300, 200);
        world->render3D(render_output, camera_pos, camera_target);
        
        // Save 3D render
        std::string filename = "3d_render_" + 
                              std::string(map_type == sim::MapType::SKYSCRAPER ? "skyscraper" :
                               map_type == sim::MapType::UNDERWATER ? "underwater" : "mountain") + ".png";
        cv::imwrite(filename, render_output);
        std::cout << "3D render saved as " << filename << std::endl;
    }
    
    std::cout << "\n=== 3D Rendering Test Completed ===" << std::endl;
    std::cout << "Check the generated PNG files for 3D renders of each map type." << std::endl;
    
    return 0;
}
