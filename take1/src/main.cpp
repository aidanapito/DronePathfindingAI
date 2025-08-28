#include "Simulator.h"
#include <iostream>
#include <opencv2/opencv.hpp>

int main(int argc, char* argv[]) {
    try {
        std::cout << "Drone Pathfinding AI Simulator" << std::endl;
        std::cout << "==============================" << std::endl;
        
        // Check OpenCV version
        std::cout << "OpenCV version: " << CV_VERSION << std::endl;
        
        // Create and configure simulator
        Simulator simulator;
        
        // Parse command line arguments
        bool use_vision_track = false;
        sim::MapType map_type = sim::MapType::MAZE;
        
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            if (arg == "--vision" || arg == "-v") {
                use_vision_track = true;
                std::cout << "Using vision-based learning track (Track B)" << std::endl;
            } else if (arg == "--maze" || arg == "-m") {
                map_type = sim::MapType::MAZE;
            } else if (arg == "--corridor" || arg == "-c") {
                map_type = sim::MapType::CORRIDOR;
            } else if (arg == "--open" || arg == "-o") {
                map_type = sim::MapType::OPEN_FIELD;
            } else if (arg == "--obstacle" || arg == "-obs") {
                map_type = sim::MapType::OBSTACLE_COURSE;
            } else if (arg == "--help" || arg == "-h") {
                std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
                std::cout << "Options:" << std::endl;
                std::cout << "  --vision, -v     Use vision-based learning (Track B)" << std::endl;
                std::cout << "  --maze, -m       Use maze map (default)" << std::endl;
                std::cout << "  --corridor, -c   Use corridor map" << std::endl;
                std::cout << "  --open, -o       Use open field map" << std::endl;
                std::cout << "  --obstacle, -obs Use obstacle course map" << std::endl;
                std::cout << "  --help, -h       Show this help message" << std::endl;
                return 0;
            }
        }
        
        // Configure simulator
        simulator.setLearningTrack(use_vision_track);
        simulator.setMapType(map_type);
        
        std::cout << "Starting simulation..." << std::endl;
        std::cout << "Press 'p' to pause/resume" << std::endl;
        std::cout << "Press 'r' to reset" << std::endl;
        std::cout << "Press 'n' for new map" << std::endl;
        std::cout << "Press 'v' to record video" << std::endl;
        std::cout << "Press 'ESC' to quit" << std::endl;
        std::cout << std::endl;
        
        // Run the simulation
        simulator.run();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown error occurred" << std::endl;
        return 1;
    }
    
    return 0;
}
