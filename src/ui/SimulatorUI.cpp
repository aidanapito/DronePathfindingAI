#include "ui/SimulatorUI.h"
#include "bridge/Environment.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>

// M_PI is already defined on macOS, so only define if not present
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ui {

SimulatorUI::SimulatorUI() 
    : window_created_(false), show_grid_(true), show_axes_(true), 
      show_path_(true), show_obstacles_(true), show_drone_(true), 
      show_goal_(true), render_mode_("3D"), mouse_dragging_(false), 
      mouse_right_dragging_(false) {
    
    // Initialize colors
    grid_color_ = cv::Scalar(100, 100, 100);
    axes_color_ = cv::Scalar(255, 255, 255);
    path_color_ = cv::Scalar(0, 255, 0);
    drone_color_ = cv::Scalar(255, 0, 0);
    goal_color_ = cv::Scalar(0, 0, 255);
    obstacle_color_ = cv::Scalar(128, 128, 128);
    background_color_ = cv::Scalar(50, 50, 100);
    
    // Initialize current frame
    current_frame_ = cv::Mat::zeros(800, 1200, CV_8UC3);
    
    std::cout << "UI initialized with default settings" << std::endl;
}

void SimulatorUI::render2D(const sim::World& world, const sim::Drone& drone, 
                           const std::vector<cv::Point2f>& path) {
    // Clear the current frame
    current_frame_ = cv::Mat::zeros(800, 1200, CV_8UC3);
    
    // Render 2D grid
    if (show_grid_) {
        render2DGrid();
    }
    
    // Render 2D obstacles
    if (show_obstacles_) {
        render2DObstacles(world);
    }
    
    // Render 2D drone
    if (show_drone_) {
        render2DDrone(drone);
    }
    
    // Render 2D path
    if (show_path_ && !path.empty()) {
        render2DPath(path);
    }
    
    // Render 2D goal
    if (show_goal_) {
        render2DGoal(world);
    }
}

void SimulatorUI::render3D(const sim::World& world, const sim::Drone& drone, 
                           const std::vector<cv::Point3f>& path_3d,
                           const std::vector<cv::Point2f>& path_2d) {
    // Get drone position to position camera properly
    cv::Point3f drone_pos = drone.getState().position;
    
    // Position camera above and behind the drone for a good view
    cv::Point3f camera_pos = drone_pos + cv::Point3f(-100, -100, 200); // Behind, to the left, and above
    cv::Point3f camera_target = drone_pos + cv::Point3f(100, 0, 0); // Looking forward from drone
    
    std::cout << "SimulatorUI render3D: Drone pos=(" << drone_pos.x << ", " << drone_pos.y << ", " << drone_pos.z << ")" << std::endl;
    std::cout << "SimulatorUI render3D: Camera pos=(" << camera_pos.x << ", " << camera_pos.y << ", " << camera_pos.z << ")" << std::endl;
    std::cout << "SimulatorUI render3D: Camera target=(" << camera_target.x << ", " << camera_target.y << ", " << camera_target.z << ")" << std::endl;
    std::cout << "SimulatorUI render3D: Current frame size: " << current_frame_.cols << "x" << current_frame_.rows << std::endl;
    
    // Create a temporary frame for the world rendering
    cv::Mat world_frame;
    world.render3D(world_frame, camera_pos, camera_target);
    
    std::cout << "SimulatorUI render3D: World frame size: " << world_frame.cols << "x" << world_frame.rows << std::endl;
    std::cout << "SimulatorUI render3D: World frame type: " << world_frame.type() << std::endl;
    
    // Check some pixel values from world frame
    if (world_frame.rows > 0 && world_frame.cols > 0) {
        cv::Vec3b world_center = world_frame.at<cv::Vec3b>(world_frame.rows/2, world_frame.cols/2);
        std::cout << "SimulatorUI render3D: World center pixel: B=" << (int)world_center[0] << " G=" << (int)world_center[1] << " R=" << (int)world_center[2] << std::endl;
    }
    
    // Resize the world frame to match our current_frame dimensions
    cv::resize(world_frame, current_frame_, current_frame_.size());
    
    std::cout << "SimulatorUI render3D: After resize, current frame size: " << current_frame_.cols << "x" << current_frame_.rows << std::endl;
    
    // Check some pixel values after resize
    if (current_frame_.rows > 0 && current_frame_.cols > 0) {
        cv::Vec3b resized_center = current_frame_.at<cv::Vec3b>(current_frame_.rows/2, current_frame_.cols/2);
        std::cout << "SimulatorUI render3D: Resized center pixel: B=" << (int)resized_center[0] << " G=" << (int)resized_center[1] << " R=" << (int)resized_center[2] << std::endl;
    }
    
    // Now overlay additional UI elements on top
    if (show_grid_) {
        render3DGrid();
    }
    
    if (show_axes_) {
        render3DAxes();
    }
    
    // Render 3D drone on top of the world
    if (show_drone_) {
        render3DDrone(drone);
    }
    
    // Render 3D path on top
    if (show_path_ && (!path_3d.empty() || !path_2d.empty())) {
        render3DPath(path_3d, path_2d);
    }
    
    // Render 3D goal on top
    if (show_goal_) {
        render3DGoal(world);
    }
    
    std::cout << "SimulatorUI render3D: Final frame size: " << current_frame_.cols << "x" << current_frame_.rows << std::endl;
}

void SimulatorUI::orbitCamera(float delta_azimuth, float delta_elevation) {
    // Convert camera position to spherical coordinates
    float dx = camera_3d_.position.x - camera_3d_.target.x;
    float dy = camera_3d_.position.y - camera_3d_.target.y;
    float dz = camera_3d_.position.z - camera_3d_.target.z;
    
    float radius = std::sqrt(dx*dx + dy*dy + dz*dz);
    float azimuth = std::atan2(dy, dx);
    float elevation = std::asin(dz / radius);
    
    // Update angles
    azimuth += delta_azimuth;
    elevation += delta_elevation;
    
    // Clamp elevation to prevent gimbal lock
    float min_elevation = -M_PI/2.0f + 0.1f;
    float max_elevation = M_PI/2.0f - 0.1f;
    elevation = std::max(min_elevation, std::min(max_elevation, elevation));
    
    // Convert back to Cartesian coordinates
    camera_3d_.position.x = camera_3d_.target.x + radius * std::cos(elevation) * std::cos(azimuth);
    camera_3d_.position.y = camera_3d_.target.y + radius * std::cos(elevation) * std::sin(azimuth);
    camera_3d_.position.z = camera_3d_.target.z + radius * std::sin(elevation);
}

void SimulatorUI::zoomCamera(float delta_distance) {
    // Calculate direction vector from target to camera
    cv::Point3f direction = camera_3d_.position - camera_3d_.target;
    float current_distance = std::sqrt(direction.x*direction.x + direction.y*direction.y + direction.z*direction.z);
    
    // Update distance
    float new_distance = std::max(10.0f, current_distance + delta_distance);
    
    // Normalize direction and scale to new distance
    direction = direction * (new_distance / current_distance);
    camera_3d_.position = camera_3d_.target + direction;
}

void SimulatorUI::panCamera(float delta_x, float delta_y) {
    // Calculate right and up vectors
    cv::Point3f direction = camera_3d_.target - camera_3d_.position;
    cv::Point3f right = direction.cross(camera_3d_.up);
    right = right * (1.0f / std::sqrt(right.x*right.x + right.y*right.y + right.z*right.z));
    
    cv::Point3f up = right.cross(direction);
    up = up * (1.0f / std::sqrt(up.x*up.x + up.y*up.y + up.z*up.z));
    
    // Pan camera and target
    float pan_speed = 10.0f;
    camera_3d_.position += right * (delta_x * pan_speed) + up * (delta_y * pan_speed);
    camera_3d_.target += right * (delta_x * pan_speed) + up * (delta_y * pan_speed);
}

void SimulatorUI::resetCamera() {
    camera_3d_ = Camera3D();
}

void SimulatorUI::createWindow(const std::string& name, int width, int height) {
    window_name_ = name;
    // Create frame with correct dimensions (width x height)
    current_frame_ = cv::Mat::zeros(height, width, CV_8UC3);
    cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
    window_created_ = true;
    
    std::cout << "Created window '" << name << "' with size " << width << "x" << height << std::endl;
    std::cout << "Frame size: " << current_frame_.cols << "x" << current_frame_.rows << std::endl;
}

void SimulatorUI::updateWindow() {
    if (window_created_) {
        cv::imshow(window_name_, current_frame_);
        cv::waitKey(1);
    }
}

void SimulatorUI::closeWindow() {
    if (window_created_) {
        cv::destroyWindow(window_name_);
        window_created_ = false;
    }
}

void SimulatorUI::handleMouseEvents() {
    // Mouse event handling would be implemented here
    // This is a placeholder for future implementation
}

void SimulatorUI::handleKeyboardEvents() {
    // Keyboard event handling would be implemented here
    // This is a placeholder for future implementation
}



void SimulatorUI::saveFrame(const std::string& filename) const {
    cv::imwrite(filename, current_frame_);
}

// Private helper methods

void SimulatorUI::render3DGrid() {
    // Render a simple 3D grid with thinner, more transparent lines
    cv::Scalar grid_color_thin(50, 50, 50); // Darker, more subtle grid
    
    for (int x = 0; x < 1200; x += 50) {
        cv::line(current_frame_, cv::Point(x, 0), cv::Point(x, 800), grid_color_thin, 1);
    }
    for (int y = 0; y < 800; y += 50) {
        cv::line(current_frame_, cv::Point(0, y), cv::Point(1200, y), grid_color_thin, 1);
    }
}

void SimulatorUI::render3DAxes() {
    // Render 3D coordinate axes
    cv::Point2f origin = project3DTo2D(cv::Point3f(0, 0, 0));
    cv::Point2f x_axis = project3DTo2D(cv::Point3f(100, 0, 0));
    cv::Point2f y_axis = project3DTo2D(cv::Point3f(0, 100, 0));
    cv::Point2f z_axis = project3DTo2D(cv::Point3f(0, 0, 100));
    
    cv::line(current_frame_, origin, x_axis, cv::Scalar(255, 0, 0), 2); // X axis (red)
    cv::line(current_frame_, origin, y_axis, cv::Scalar(0, 255, 0), 2); // Y axis (green)
    cv::line(current_frame_, origin, z_axis, cv::Scalar(0, 0, 255), 2); // Z axis (blue)
}

void SimulatorUI::render3DObstacles(const sim::World& world) {
    // Get obstacles from world and render them in 3D
    const auto& obstacles = world.getObstacles();
    for (const auto& obs : obstacles) {
        cv::Point2f screen_pos = project3DTo2D(obs.position);
        if (screen_pos.x >= 0 && screen_pos.x < 1200 && screen_pos.y >= 0 && screen_pos.y < 800) {
            float screen_radius = obs.radius * 2.0f; // Scale for visibility
            cv::circle(current_frame_, screen_pos, screen_radius, obstacle_color_, -1);
        }
    }
}

void SimulatorUI::render3DDrone(const sim::Drone& drone) {
    // Get drone position and render it in 3D
    cv::Point3f drone_pos = drone.getState().position;
    cv::Point2f screen_pos = project3DTo2D(drone_pos);
    if (screen_pos.x >= 0 && screen_pos.x < 1200 && screen_pos.y >= 0 && screen_pos.y < 800) {
        cv::circle(current_frame_, screen_pos, DRONE_SIZE, drone_color_, -1);
    }
}

void SimulatorUI::render3DPath(const std::vector<cv::Point3f>& path_3d, 
                               const std::vector<cv::Point2f>& path_2d) {
    // Render 3D path
    if (!path_3d.empty()) {
        for (size_t i = 1; i < path_3d.size(); ++i) {
            cv::Point2f start = project3DTo2D(path_3d[i-1]);
            cv::Point2f end = project3DTo2D(path_3d[i]);
            if (start.x >= 0 && start.x < 1200 && start.y >= 0 && start.y < 800 &&
                end.x >= 0 && end.x < 1200 && end.y >= 0 && end.y < 800) {
                cv::line(current_frame_, start, end, path_color_, 2);
            }
        }
    }
    
    // Render 2D path (top-down view)
    if (!path_2d.empty()) {
        for (size_t i = 1; i < path_2d.size(); ++i) {
            cv::Point2f start = path_2d[i-1];
            cv::Point2f end = path_2d[i];
            cv::line(current_frame_, start, end, cv::Scalar(0, 255, 255), 1);
        }
    }
}

void SimulatorUI::render3DGoal(const sim::World& world) {
    // Get goal position and render it in 3D
    cv::Point3f goal_pos = world.getGoalPosition();
    cv::Point2f screen_pos = project3DTo2D(goal_pos);
    if (screen_pos.x >= 0 && screen_pos.x < 1200 && screen_pos.y >= 0 && screen_pos.y < 800) {
        cv::circle(current_frame_, screen_pos, GOAL_SIZE, goal_color_, -1);
    }
}

void SimulatorUI::render2DGrid() {
    // Render a simple 2D grid
    for (int x = 0; x < 1200; x += 50) {
        cv::line(current_frame_, cv::Point(x, 0), cv::Point(x, 800), grid_color_, 1);
    }
    for (int y = 0; y < 800; y += 50) {
        cv::line(current_frame_, cv::Point(0, y), cv::Point(1200, y), grid_color_, 1);
    }
}

void SimulatorUI::render2DObstacles(const sim::World& world) {
    // Get obstacles from world and render them in 2D
    const auto& obstacles = world.getObstacles();
    for (const auto& obs : obstacles) {
        cv::Point2f pos(obs.position.x, obs.position.y);
        if (pos.x >= 0 && pos.x < 1200 && pos.y >= 0 && pos.y < 800) {
            cv::circle(current_frame_, pos, obs.radius, obstacle_color_, -1);
        }
    }
}

void SimulatorUI::render2DDrone(const sim::Drone& drone) {
    // Get drone position and render it in 2D
    cv::Point3f drone_pos = drone.getState().position;
    cv::Point2f pos(drone_pos.x, drone_pos.y);
    if (pos.x >= 0 && pos.x < 1200 && pos.y >= 0 && pos.y < 800) {
        cv::circle(current_frame_, pos, DRONE_SIZE, drone_color_, -1);
    }
}

void SimulatorUI::render2DPath(const std::vector<cv::Point2f>& path) {
    // Render 2D path
    for (size_t i = 1; i < path.size(); ++i) {
        cv::line(current_frame_, path[i-1], path[i], path_color_, 2);
    }
}

void SimulatorUI::render2DGoal(const sim::World& world) {
    // Get goal position and render it in 2D
    cv::Point3f goal_pos = world.getGoalPosition();
    cv::Point2f pos(goal_pos.x, goal_pos.y);
    if (pos.x >= 0 && pos.x < 1200 && pos.y >= 0 && pos.y < 800) {
        cv::circle(current_frame_, pos, GOAL_SIZE, goal_color_, -1);
    }
}

cv::Point2f SimulatorUI::project3DTo2D(const cv::Point3f& point_3d) const {
    // Simple perspective projection
    float depth = point_3d.z - camera_3d_.position.z;
    if (depth <= 0) return cv::Point2f(-1, -1); // Behind camera
    
    float scale = 1000.0f / (depth + 100.0f);
    float screen_x = (point_3d.x - camera_3d_.position.x) * scale + 600;
    float screen_y = (point_3d.y - camera_3d_.position.y) * scale + 400;
    
    return cv::Point2f(screen_x, screen_y);
}

cv::Point3f SimulatorUI::screenTo3D(const cv::Point2f& screen_point, float depth) const {
    // Inverse perspective projection
    float scale = 1000.0f / (depth + 100.0f);
    float world_x = (screen_point.x - 600) / scale + camera_3d_.position.x;
    float world_y = (screen_point.y - 400) / scale + camera_3d_.position.y;
    float world_z = depth + camera_3d_.position.z;
    
    return cv::Point3f(world_x, world_y, world_z);
}

cv::Mat SimulatorUI::getViewMatrix() const {
    // Simple view matrix calculation
    cv::Mat view_matrix = cv::Mat::eye(4, 4, CV_32F);
    
    // This is a simplified view matrix - in a real implementation,
    // you would calculate the proper rotation and translation
    return view_matrix;
}

cv::Mat SimulatorUI::getProjectionMatrix() const {
    // Simple projection matrix calculation
    cv::Mat proj_matrix = cv::Mat::eye(4, 4, CV_32F);
    
    // This is a simplified projection matrix - in a real implementation,
    // you would calculate the proper perspective projection
    return proj_matrix;
}

} // namespace ui
