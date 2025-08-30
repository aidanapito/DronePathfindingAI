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
    Obstacle skyscraper;
    skyscraper.position = cv::Point3f(x, y, 50);  // Base height 50
    skyscraper.radius = 20.0f;
    skyscraper.height = 100.0f;
    skyscraper.is_vertical = true;
    skyscraper.color = cv::Scalar(100, 100, 100);  // Gray
    skyscraper.type = ObstacleType::SKYSCRAPER;
    obstacles_.push_back(skyscraper);
}

void World::generateGroundObstacle(float x, float y) {
    Obstacle obstacle;
    obstacle.position = cv::Point3f(x, y, 25);  // Half height for ground
    obstacle.radius = 15.0f;
    obstacle.height = 20.0f;
    obstacle.is_vertical = false;
    obstacle.color = cv::Scalar(139, 69, 19);  // Brown
    obstacle.type = ObstacleType::GROUND_OBSTACLE;
    obstacles_.push_back(obstacle);
}

void World::generateMountain(float x, float y, float height) {
    Obstacle mountain;
    mountain.position = cv::Point3f(x, y, height/2);  // Center at half height
    mountain.radius = 40.0f;
    mountain.height = height;
    mountain.is_vertical = false;
    mountain.color = cv::Scalar(105, 105, 105);  // Dark gray
    mountain.type = ObstacleType::MOUNTAIN;
    obstacles_.push_back(mountain);
}

void World::render3D(cv::Mat& output, const cv::Point3f& camera_pos, const cv::Point3f& camera_target) {
    std::cout << "World::render3D: " << obstacles_.size() << " obstacles, camera at (" 
              << camera_pos.x << ", " << camera_pos.y << ", " << camera_pos.z << ")" << std::endl;
    
    // Calculate camera orientation
    cv::Point3f camera_forward = camera_target - camera_pos;
    float camera_distance = sqrt(camera_forward.x * camera_forward.x + 
                                camera_forward.y * camera_forward.y + 
                                camera_forward.z * camera_forward.z);
    camera_forward = camera_forward / camera_distance;
    
    // Camera up vector (always pointing up)
    cv::Point3f camera_up(0, 0, 1);
    
    // Camera right vector (perpendicular to forward and up)
    cv::Point3f camera_right = camera_forward.cross(camera_up);
    camera_right = camera_right / sqrt(camera_right.x * camera_right.x + 
                                      camera_right.y * camera_right.y + 
                                      camera_right.z * camera_right.z);
    
    // Recalculate up vector to be perpendicular to both forward and right
    camera_up = camera_right.cross(camera_forward);
    
    // Find obstacles in front of camera
    std::vector<std::pair<float, const Obstacle*>> visible_obstacles;
    
    for (const auto& obstacle : obstacles_) {
        // Vector from camera to obstacle
        cv::Point3f to_obstacle = obstacle.position - camera_pos;
        float distance = sqrt(to_obstacle.x * to_obstacle.x + 
                             to_obstacle.y * to_obstacle.y + 
                             to_obstacle.z * to_obstacle.z);
        
        // Check if obstacle is in front of camera (dot product > 0)
        float dot_product = to_obstacle.x * camera_forward.x + 
                           to_obstacle.y * camera_forward.y + 
                           to_obstacle.z * camera_forward.z;
        
        if (dot_product > 0 && distance < 1000.0f) { // Only render obstacles within 1000 units
            visible_obstacles.push_back({distance, &obstacle});
        }
    }
    
    std::cout << "World::render3D: " << visible_obstacles.size() << " obstacles in front of camera" << std::endl;
    
    // Sort obstacles by depth (back to front for proper rendering)
    std::sort(visible_obstacles.begin(), visible_obstacles.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });
    
    // Render obstacles from back to front
    for (const auto& [depth, obstacle] : visible_obstacles) {
        render3DObject(output, *obstacle, camera_pos, camera_forward, camera_right, camera_up);
    }
    
    // Render ground plane for better 3D effect
    renderGroundPlane(output, camera_pos, camera_forward, camera_right, camera_up);
}

void World::render3DObject(cv::Mat& output, const Obstacle& obstacle, 
                          const cv::Point3f& camera_pos, const cv::Point3f& camera_forward,
                          const cv::Point3f& camera_right, const cv::Point3f& camera_up) {
    // Calculate 3D object dimensions based on type
    float width, height, depth;
    cv::Scalar color;
    
    switch (obstacle.type) {
        case ObstacleType::SKYSCRAPER:
            width = 40.0f;
            height = 100.0f;
            depth = 40.0f;
            color = cv::Scalar(100, 100, 100); // Gray
            break;
        case ObstacleType::GROUND_OBSTACLE:
            width = 30.0f;
            height = 20.0f;
            depth = 30.0f;
            color = cv::Scalar(139, 69, 19); // Brown
            break;
        case ObstacleType::MOUNTAIN:
            width = 80.0f;
            height = 60.0f;
            depth = 80.0f;
            color = cv::Scalar(105, 105, 105); // Dark gray
            break;
        default:
            width = 20.0f;
            height = 20.0f;
            depth = 20.0f;
            color = cv::Scalar(255, 255, 255); // White
            break;
    }
    
    // Calculate 3D bounding box corners
    std::vector<cv::Point3f> corners = {
        cv::Point3f(obstacle.position.x - width/2, obstacle.position.y - depth/2, obstacle.position.z),
        cv::Point3f(obstacle.position.x + width/2, obstacle.position.y - depth/2, obstacle.position.z),
        cv::Point3f(obstacle.position.x + width/2, obstacle.position.y + depth/2, obstacle.position.z),
        cv::Point3f(obstacle.position.x - width/2, obstacle.position.y + depth/2, obstacle.position.z),
        cv::Point3f(obstacle.position.x - width/2, obstacle.position.y - depth/2, obstacle.position.z + height),
        cv::Point3f(obstacle.position.x + width/2, obstacle.position.y - depth/2, obstacle.position.z + height),
        cv::Point3f(obstacle.position.x + width/2, obstacle.position.y + depth/2, obstacle.position.z + height),
        cv::Point3f(obstacle.position.x - width/2, obstacle.position.y + depth/2, obstacle.position.z + height)
    };
    
    // Project all corners to 2D
    std::vector<cv::Point2f> projected_corners;
    for (const auto& corner : corners) {
        cv::Point2f projected = project3DTo2D(corner, camera_pos, camera_forward);
        projected_corners.push_back(projected);
    }
    
    // Render 3D object faces
    render3DFace(output, projected_corners, {0, 1, 2, 3}, color); // Bottom face
    render3DFace(output, projected_corners, {4, 5, 6, 7}, color); // Top face
    render3DFace(output, projected_corners, {0, 1, 5, 4}, color); // Front face
    render3DFace(output, projected_corners, {2, 3, 7, 6}, color); // Back face
    render3DFace(output, projected_corners, {0, 3, 7, 4}, color); // Left face
    render3DFace(output, projected_corners, {1, 2, 6, 5}, color); // Right face
    
    // Add windows for skyscrapers
    if (obstacle.type == ObstacleType::SKYSCRAPER) {
        renderSkyscraperWindows(output, projected_corners, obstacle);
    }
}

void World::render3DFace(cv::Mat& output, const std::vector<cv::Point2f>& corners, 
                        const std::vector<int>& face_indices, const cv::Scalar& color) {
    // Check if all corners are visible on screen
    bool all_visible = true;
    for (int idx : face_indices) {
        if (corners[idx].x < 0 || corners[idx].x >= output.cols ||
            corners[idx].y < 0 || corners[idx].y >= output.rows) {
            all_visible = false;
            break;
        }
    }
    
    if (!all_visible) return;
    
    // Create polygon points for the face
    std::vector<cv::Point> polygon_points;
    for (int idx : face_indices) {
        polygon_points.push_back(cv::Point(static_cast<int>(corners[idx].x), 
                                         static_cast<int>(corners[idx].y)));
    }
    
    // Fill the face with color
    cv::fillPoly(output, std::vector<std::vector<cv::Point>>{polygon_points}, color);
    
    // Draw edges in darker color
    cv::Scalar edge_color = color * 0.7;
    for (size_t i = 0; i < face_indices.size(); ++i) {
        int idx1 = face_indices[i];
        int idx2 = face_indices[(i + 1) % face_indices.size()];
        cv::line(output, corners[idx1], corners[idx2], edge_color, 2);
    }
}

void World::renderSkyscraperWindows(cv::Mat& output, const std::vector<cv::Point2f>& corners, 
                                   const Obstacle& obstacle) {
    // Calculate window positions on the front face
    cv::Point2f front_center = (corners[0] + corners[1] + corners[5] + corners[4]) / 4.0f;
    
    // Add windows in a grid pattern
    for (int row = 0; row < 5; ++row) {
        for (int col = 0; col < 3; ++col) {
            float window_x = front_center.x + (col - 1) * 15.0f;
            float window_y = front_center.y + (row - 2) * 20.0f;
            
            cv::Point2f window_pos(window_x, window_y);
            
            // Check if window is visible
            if (window_pos.x >= 0 && window_pos.x < output.cols &&
                window_pos.y >= 0 && window_pos.y < output.rows) {
                cv::circle(output, window_pos, 3, cv::Scalar(255, 255, 200), -1); // Yellow windows
            }
        }
    }
}

void World::renderGroundPlane(cv::Mat& output, const cv::Point3f& camera_pos, 
                             const cv::Point3f& camera_forward, const cv::Point3f& camera_right, 
                             const cv::Point3f& camera_up) {
    // Create a large ground plane
    float ground_size = 2000.0f;
    float ground_height = 0.0f;
    
    // Calculate ground corners in world space
    std::vector<cv::Point3f> ground_corners = {
        cv::Point3f(-ground_size, -ground_size, ground_height),
        cv::Point3f(ground_size, -ground_size, ground_height),
        cv::Point3f(ground_size, ground_size, ground_height),
        cv::Point3f(-ground_size, ground_size, ground_height)
    };
    
    // Project ground corners to 2D
    std::vector<cv::Point2f> projected_ground;
    for (const auto& corner : ground_corners) {
        cv::Point2f projected = project3DTo2D(corner, camera_pos, camera_forward);
        projected_ground.push_back(projected);
    }
    
    // Draw ground with grid pattern
    cv::Scalar ground_color(34, 139, 34); // Forest green
    cv::fillPoly(output, std::vector<std::vector<cv::Point>>{std::vector<cv::Point>{
        cv::Point(static_cast<int>(projected_ground[0].x), static_cast<int>(projected_ground[0].y)),
        cv::Point(static_cast<int>(projected_ground[1].x), static_cast<int>(projected_ground[1].y)),
        cv::Point(static_cast<int>(projected_ground[2].x), static_cast<int>(projected_ground[2].y)),
        cv::Point(static_cast<int>(projected_ground[3].x), static_cast<int>(projected_ground[3].y))
    }}, ground_color);
    
    // Draw grid lines on ground
    cv::Scalar grid_color(50, 205, 50); // Lime green
    for (int i = -10; i <= 10; ++i) {
        float x = i * 200.0f;
        float y = i * 200.0f;
        
        // X-direction lines
        cv::Point3f line_start1(x, -ground_size, ground_height);
        cv::Point3f line_end1(x, ground_size, ground_height);
        cv::Point2f proj_start1 = project3DTo2D(line_start1, camera_pos, camera_forward);
        cv::Point2f proj_end1 = project3DTo2D(line_end1, camera_pos, camera_forward);
        
        if (proj_start1.x >= 0 && proj_start1.x < output.cols && proj_start1.y >= 0 && proj_start1.y < output.rows &&
            proj_end1.x >= 0 && proj_end1.x < output.cols && proj_end1.y >= 0 && proj_end1.y < output.rows) {
            cv::line(output, proj_start1, proj_end1, grid_color, 1);
        }
        
        // Y-direction lines
        cv::Point3f line_start2(-ground_size, y, ground_height);
        cv::Point3f line_end2(ground_size, y, ground_height);
        cv::Point2f proj_start2 = project3DTo2D(line_start2, camera_pos, camera_forward);
        cv::Point2f proj_end2 = project3DTo2D(line_end2, camera_pos, camera_forward);
        
        if (proj_start2.x >= 0 && proj_start2.x < output.cols && proj_start2.y >= 0 && proj_start2.y < output.rows &&
            proj_end2.x >= 0 && proj_end2.x < output.cols && proj_end2.y >= 0 && proj_end2.y < output.rows) {
            cv::line(output, proj_start2, proj_end2, grid_color, 1);
        }
    }
}

cv::Point2f World::project3DTo2D(const cv::Point3f& point3d, 
                                  const cv::Point3f& camera_pos,
                                  const cv::Point3f& camera_target) {
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
