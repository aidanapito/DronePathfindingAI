#include "World.h"
#include <cmath>
#include <algorithm>

World::World(int width, int height, int depth) 
    : width_(width), height_(height), depth_(depth) {
    // Initialize random number generator
    std::random_device rd;
    rng_.seed(rd());
}

void World::generateTerrain() {
    // Clear existing obstacles
    obstacles_.clear();
    
    // Add skyscrapers
    addSkyscrapers(15);
    
    // Add ground obstacles
    addGroundObstacles(20);
    
    // Add mountain pass
    addMountainPass();
    
    std::cout << "🌍 Generated world with " << obstacles_.size() << " obstacles" << std::endl;
}

void World::addSkyscrapers(int count) {
    std::uniform_real_distribution<float> x_dist(100.0f, width_ - 100.0f);
    std::uniform_real_distribution<float> y_dist(100.0f, height_ - 100.0f);
    std::uniform_real_distribution<float> height_dist(80.0f, 200.0f);
    
    for (int i = 0; i < count; ++i) {
        generateSkyscraper(x_dist(rng_), y_dist(rng_));
    }
}

void World::addGroundObstacles(int count) {
    std::uniform_real_distribution<float> x_dist(50.0f, width_ - 50.0f);
    std::uniform_real_distribution<float> y_dist(50.0f, height_ - 50.0f);
    std::uniform_real_distribution<float> radius_dist(10.0f, 30.0f);
    
    for (int i = 0; i < count; ++i) {
        generateGroundObstacle(x_dist(rng_), y_dist(rng_));
    }
}

void World::addMountainPass() {
    // Create a mountain range along the Y axis
    for (int y = 100; y < height_ - 100; y += 80) {
        float mountain_height = 120.0f + (rng_() % 80);
        generateMountain(width_ / 2, y, mountain_height);
    }
}

void World::generateSkyscraper(float x, float y) {
    Obstacle obs;
    obs.position = cv::Point3f(x, y, 50.0f); // Buildings start at ground level + 50
    obs.radius = 15.0f + (rng_() % 20);     // 15-35 unit radius
    obs.height = 80.0f + (rng_() % 120);    // 80-200 unit height
    obs.is_vertical = true;
    obs.color = cv::Scalar(100, 100, 100);  // Gray buildings
    
    obstacles_.push_back(obs);
}

void World::generateGroundObstacle(float x, float y) {
    Obstacle obs;
    obs.position = cv::Point3f(x, y, 25.0f); // Ground obstacles at 25 units height
    obs.radius = 8.0f + (rng_() % 15);      // 8-23 unit radius
    obs.height = 50.0f + (rng_() % 50);     // 50-100 unit height
    obs.is_vertical = false;
    obs.color = cv::Scalar(139, 69, 19);    // Brown obstacles
    
    obstacles_.push_back(obs);
}

void World::generateMountain(float x, float y, float height) {
    Obstacle obs;
    obs.position = cv::Point3f(x, y, height / 2); // Mountain center at half height
    obs.radius = 25.0f + (rng_() % 20);          // 25-45 unit radius
    obs.height = height;
    obs.is_vertical = false;
    obs.color = cv::Scalar(105, 105, 105);       // Dark gray mountains
    
    obstacles_.push_back(obs);
}

void World::render3D(cv::Mat& output, const cv::Point3f& camera_pos, 
                     const cv::Point3f& camera_target) const {
    // Create sky blue background
    output = cv::Mat(height_, width_, CV_8UC3, cv::Scalar(135, 206, 235));
    
    // Debug: Print rendering info
    std::cout << "World::render3D: " << obstacles_.size() << " obstacles, camera at (" 
              << camera_pos.x << ", " << camera_pos.y << ", " << camera_pos.z << ")" << std::endl;
    
    // Calculate camera direction
    cv::Point3f camera_dir = camera_target - camera_pos;
    float camera_distance = sqrt(camera_dir.x * camera_dir.x + 
                                camera_dir.y * camera_dir.y + 
                                camera_dir.z * camera_dir.z);
    camera_dir /= camera_distance;
    
    // Render obstacles with depth sorting (back to front)
    std::vector<std::pair<float, const Obstacle*>> sorted_obstacles;
    
    for (const auto& obs : obstacles_) {
        float depth = camera_pos.z - obs.position.z;
        if (depth > 0) { // Only render obstacles in front of camera
            sorted_obstacles.push_back({depth, &obs});
        }
    }
    
    // Sort by depth (farthest first)
    std::sort(sorted_obstacles.begin(), sorted_obstacles.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });
    
    // Debug: Print how many obstacles are visible
    std::cout << "World::render3D: " << sorted_obstacles.size() << " obstacles in front of camera" << std::endl;
    
    // Render obstacles from back to front
    int rendered_count = 0;
    for (const auto& [depth, obs] : sorted_obstacles) {
        cv::Point2f screen_pos = project3DTo2D(obs->position, camera_pos, camera_target);
        
        // Debug: Print first few obstacle projections
        if (rendered_count < 5) {
            std::cout << "Obstacle " << rendered_count << ": 3D(" << obs->position.x << ", " << obs->position.y << ", " << obs->position.z 
                      << ") -> 2D(" << screen_pos.x << ", " << screen_pos.y << ") depth=" << depth << std::endl;
        }
        
        if (screen_pos.x >= 0 && screen_pos.x < width_ && 
            screen_pos.y >= 0 && screen_pos.y < height_) {
            
            // Calculate scale based on depth - much more conservative scaling
            float scale = 200.0f / (depth + 100.0f);
            float screen_radius = obs->radius * scale;
            float screen_height = obs->height * scale;
            
            if (obs->is_vertical) {
                // Draw skyscrapers as rectangles
                cv::Point2f top_left(screen_pos.x - screen_radius, screen_pos.y - screen_height);
                cv::Point2f bottom_right(screen_pos.x + screen_radius, screen_pos.y);
                
                // Draw building outline
                cv::rectangle(output, top_left, bottom_right, obs->color, 2);
                
                // Draw windows (small yellow dots)
                for (int w = 0; w < 3; ++w) {
                    for (int h = 0; h < 5; ++h) {
                        float window_x = screen_pos.x - screen_radius/2 + w * screen_radius/2;
                        float window_y = screen_pos.y - screen_height/2 + h * screen_height/5;
                        cv::circle(output, cv::Point2f(window_x, window_y), 2, 
                                 cv::Scalar(255, 255, 200), -1);
                    }
                }
            } else {
                // Draw ground obstacles as circles
                cv::circle(output, screen_pos, screen_radius, obs->color, 2);
            }
        }
    }
}

cv::Point2f World::project3DTo2D(const cv::Point3f& point3d, 
                                  const cv::Point3f& camera_pos,
                                  const cv::Point3f& camera_target) const {
    // Calculate depth
    float depth = camera_pos.z - point3d.z;
    if (depth <= 0) return cv::Point2f(-1, -1); // Behind camera
    
    // Calculate scale based on depth - much more conservative scaling
    float scale = 200.0f / (depth + 100.0f);
    
    // Project to screen coordinates
    float screen_x = (point3d.x - camera_pos.x) * scale + width_ / 2;
    float screen_y = (point3d.y - camera_pos.y) * scale + height_ / 2;
    
    return cv::Point2f(screen_x, screen_y);
}

bool World::checkCollision(const cv::Point3f& position, float radius) const {
    for (const auto& obs : obstacles_) {
        float dx = position.x - obs.position.x;
        float dy = position.y - obs.position.y;
        float dz = position.z - obs.position.z;
        float distance = sqrt(dx*dx + dy*dy + dz*dz);
        
        if (distance < (radius + obs.radius)) {
            return true; // Collision detected
        }
    }
    return false;
}

float World::getGroundHeight(float x, float y) const {
    // Simple ground height - could be enhanced with terrain generation
    return 0.0f;
}
