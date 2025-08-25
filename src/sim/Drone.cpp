#include "sim/Drone.h"
#include "sim/World.h"
#include <cmath>

namespace sim {

Drone::Drone(const cv::Point2f& start_pos, float start_heading)
    : world_(nullptr), camera_height_(100.0f), camera_fov_(M_PI / 3.0f), max_sensor_range_(200.0f) {
    state_.position = start_pos;
    state_.heading = start_heading;
    state_.velocity = 0.0f;
    state_.angular_velocity = 0.0f;
    
    // Set default constraints
    constraints_.max_velocity = 100.0f;
    constraints_.max_angular_velocity = M_PI / 2.0f;
    constraints_.min_turn_radius = 20.0f;
    constraints_.safety_margin = 15.0f;
    constraints_.emergency_stop_dist = 10.0f;
}

void Drone::update(float dt, float throttle, float yaw_rate) {
    // Apply constraints
    if (!isWithinConstraints(throttle, yaw_rate)) {
        throttle = 0.0f;
        yaw_rate = 0.0f;
    }
    
    // Update kinematics
    state_.angular_velocity = yaw_rate;
    state_.heading += state_.angular_velocity * dt;
    
    // Normalize heading to [0, 2π)
    while (state_.heading < 0) state_.heading += 2.0f * M_PI;
    while (state_.heading >= 2.0f * M_PI) state_.heading -= 2.0f * M_PI;
    
    // Update velocity
    state_.velocity = throttle * constraints_.max_velocity;
    
    // Update position
    float dx = state_.velocity * cos(state_.heading) * dt;
    float dy = state_.velocity * sin(state_.heading) * dt;
    state_.position += cv::Point2f(dx, dy);
}

bool Drone::isEmergencyStop() const {
    // Check if we're too close to any obstacle
    if (world_) {
        return world_->checkCollision(state_.position, constraints_.emergency_stop_dist);
    }
    return false;
}

void Drone::emergencyStop() {
    state_.velocity = 0.0f;
    state_.angular_velocity = 0.0f;
}

bool Drone::isWithinConstraints(float throttle, float yaw_rate) const {
    if (std::abs(throttle) > 1.0f) return false;
    if (std::abs(yaw_rate) > constraints_.max_angular_velocity) return false;
    
    // Check minimum turn radius only when moving forward
    if (throttle > 0.1f && std::abs(yaw_rate) > 0) {
        float turn_radius = state_.velocity / std::abs(yaw_rate);
        if (turn_radius < constraints_.min_turn_radius) return false;
    }
    
    return true;
}

cv::Mat Drone::getTopDownView(const World& world, int view_size) const {
    // TODO: Implement top-down view rendering
    cv::Mat view(view_size, view_size, CV_8UC3, cv::Scalar(128, 128, 128));
    return view;
}

cv::Mat Drone::getEgoView(const World& world, int view_size) const {
    // TODO: Implement ego-centric view rendering
    cv::Mat view(view_size, view_size, CV_8UC3, cv::Scalar(128, 128, 128));
    return view;
}

cv::Mat Drone::getDepthMap(const World& world, int num_rays) const {
    // TODO: Implement depth map generation
    cv::Mat depth_map(num_rays, 1, CV_32F, cv::Scalar(max_sensor_range_));
    return depth_map;
}

cv::Mat Drone::getOccupancyGrid(const World& world, int grid_size) const {
    // TODO: Implement occupancy grid generation
    cv::Mat grid(grid_size, grid_size, CV_8UC1, cv::Scalar(0));
    return grid;
}

void Drone::setState(const DroneState& state) {
    state_ = state;
}

void Drone::setConstraints(const DroneConstraints& constraints) {
    constraints_ = constraints;
}

float Drone::getDistanceToGoal(const cv::Point2f& goal) const {
    return cv::norm(state_.position - goal);
}

float Drone::getProgressToGoal(const cv::Point2f& start, const cv::Point2f& goal) const {
    cv::Point2f start_to_goal = goal - start;
    cv::Point2f start_to_current = state_.position - start;
    
    float total_distance = cv::norm(start_to_goal);
    if (total_distance < 1e-6f) return 0.0f;
    
    float progress = start_to_goal.dot(start_to_current) / (total_distance * total_distance);
    return std::max(0.0f, std::min(1.0f, progress));
}

bool Drone::hasReachedGoal(const cv::Point2f& goal, float threshold) const {
    return getDistanceToGoal(goal) < threshold;
}

cv::Mat Drone::renderRaycast(const World& world, float start_angle, float end_angle, int num_rays) const {
    // TODO: Implement raycast rendering
    cv::Mat raycast(num_rays, 1, CV_32F, cv::Scalar(max_sensor_range_));
    return raycast;
}

cv::Point2f Drone::worldToCamera(const cv::Point2f& world_point) const {
    // TODO: Implement world to camera coordinate transformation
    return world_point - state_.position;
}

cv::Point2f Drone::cameraToWorld(const cv::Point2f& camera_point) const {
    // TODO: Implement camera to world coordinate transformation
    return state_.position + camera_point;
}

} // namespace sim
