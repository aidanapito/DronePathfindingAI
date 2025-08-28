#include "sim/Drone.h"
#include "sim/World.h"
#include <cmath>
#include <algorithm>

namespace sim {

Drone::Drone(const cv::Point3f& start_pos, float start_heading, float start_pitch, float start_roll)
    : world_(nullptr), camera_height_(100.0f), camera_fov_(M_PI / 3.0f), max_sensor_range_(200.0f) {
    state_.position = start_pos;
    state_.heading = start_heading;
    state_.pitch = start_pitch;
    state_.roll = start_roll;
    state_.velocity = 0.0f;
    state_.angular_velocity = 0.0f;
    state_.pitch_rate = 0.0f;
    state_.roll_rate = 0.0f;
    state_.vertical_velocity = 0.0f;
    
    // Set default constraints
    constraints_.max_velocity = 100.0f;
    constraints_.max_angular_velocity = M_PI / 2.0f;
    constraints_.max_pitch_rate = M_PI / 4.0f;
    constraints_.max_roll_rate = M_PI / 4.0f;
    constraints_.max_vertical_velocity = 50.0f;
    constraints_.min_turn_radius = 20.0f;
    constraints_.safety_margin = 15.0f;
    constraints_.emergency_stop_dist = 10.0f;
    constraints_.max_altitude = 400.0f;
    constraints_.min_altitude = 0.0f;
}

void Drone::update(float dt, float throttle, float yaw_rate, float pitch_rate, 
                   float roll_rate, float vertical_thrust) {
    // Apply constraints
    if (!isWithinConstraints(throttle, yaw_rate, pitch_rate, roll_rate, vertical_thrust)) {
        throttle = 0.0f;
        yaw_rate = 0.0f;
        pitch_rate = 0.0f;
        roll_rate = 0.0f;
        vertical_thrust = 0.0f;
    }
    
    // Update 3D kinematics
    update3DKinematics(dt, throttle, yaw_rate, pitch_rate, roll_rate, vertical_thrust);
    
    // Check if new position would be valid
    cv::Point3f new_position = calculate3DPosition(dt, throttle, yaw_rate, pitch_rate, roll_rate, vertical_thrust);
    bool can_move = true;
    
    // Check collision with obstacles
    if (world_ && check3DCollision(new_position, constraints_.safety_margin)) {
        can_move = false;
    }
    
    // Check if new position would be out of bounds
    if (world_ && !world_->isInBounds(new_position)) {
        can_move = false;
    }
    
    // Check altitude bounds
    if (new_position.z < constraints_.min_altitude || new_position.z > constraints_.max_altitude) {
        can_move = false;
    }
    
    // Check if new position would be too close to boundaries
    if (world_) {
        float margin = constraints_.safety_margin;
        auto size = world_->getSize();
        if (new_position.x < margin || new_position.x >= size.width - margin ||
            new_position.y < margin || new_position.y >= size.height - margin ||
            new_position.z < margin || new_position.z >= size.depth - margin) {
            can_move = false;
        }
    }
    
    // Update position only if movement is valid
    if (can_move) {
        state_.position = new_position;
    } else {
        // If we can't move, reduce velocity to simulate hitting something
        state_.velocity *= 0.5f;
        state_.vertical_velocity *= 0.5f;
        
        // Clamp position to world bounds if we somehow got outside
        if (world_) {
            float margin = constraints_.safety_margin;
            auto size = world_->getSize();
            state_.position.x = std::max(margin, std::min(size.width - margin, state_.position.x));
            state_.position.y = std::max(margin, std::min(size.height - margin, state_.position.y));
            state_.position.z = std::max(constraints_.min_altitude + margin, 
                                       std::min(size.depth - margin, state_.position.z));
        }
    }
}

void Drone::update3DKinematics(float dt, float throttle, float yaw_rate, float pitch_rate, 
                              float roll_rate, float vertical_thrust) {
    // Update angular velocities
    state_.angular_velocity = yaw_rate;
    state_.pitch_rate = pitch_rate;
    state_.roll_rate = roll_rate;
    
    // Update angles
    state_.heading += state_.angular_velocity * dt;
    state_.pitch += state_.pitch_rate * dt;
    state_.roll += state_.roll_rate * dt;
    
    // Normalize angles to [0, 2π) for heading, [-π, π] for pitch and roll
    while (state_.heading < 0) state_.heading += 2.0f * M_PI;
    while (state_.heading >= 2.0f * M_PI) state_.heading -= 2.0f * M_PI;
    
    while (state_.pitch < -M_PI) state_.pitch += 2.0f * M_PI;
    while (state_.pitch > M_PI) state_.pitch -= 2.0f * M_PI;
    
    while (state_.roll < -M_PI) state_.roll += 2.0f * M_PI;
    while (state_.roll > M_PI) state_.roll -= 2.0f * M_PI;
    
    // Update velocities
    state_.velocity = throttle * constraints_.max_velocity;
    state_.vertical_velocity = vertical_thrust * constraints_.max_vertical_velocity;
}

cv::Point3f Drone::calculate3DPosition(float dt, float throttle, float yaw_rate, 
                                       float pitch_rate, float roll_rate, float vertical_thrust) const {
    // Calculate new angles
    float new_heading = state_.heading + yaw_rate * dt;
    float new_pitch = state_.pitch + pitch_rate * dt;
    float new_roll = state_.roll + roll_rate * dt;
    
    // Calculate new velocities
    float new_velocity = throttle * constraints_.max_velocity;
    float new_vertical_velocity = vertical_thrust * constraints_.max_vertical_velocity;
    
    // Calculate new position based on 3D kinematics
    float dx = new_velocity * cos(new_heading) * cos(new_pitch) * dt;
    float dy = new_velocity * sin(new_heading) * cos(new_pitch) * dt;
    float dz = new_vertical_velocity * dt + new_velocity * sin(new_pitch) * dt;
    
    return state_.position + cv::Point3f(dx, dy, dz);
}

bool Drone::check3DCollision(const cv::Point3f& position, float radius) const {
    if (!world_) return false;
    return world_->checkCollision(position, radius);
}

bool Drone::isEmergencyStop() const {
    // Check if we're too close to any obstacle
    if (world_) {
        // Check for obstacle collisions
        if (check3DCollision(state_.position, constraints_.emergency_stop_dist)) {
            return true;
        }
        
        // Check if we're too close to map boundaries
        float margin = constraints_.emergency_stop_dist;
        auto size = world_->getSize();
        if (state_.position.x < margin || state_.position.x >= size.width - margin ||
            state_.position.y < margin || state_.position.y >= size.height - margin ||
            state_.position.z < margin || state_.position.z >= size.depth - margin) {
            return true;
        }
    }
    
    // Check altitude bounds
    if (state_.position.z < constraints_.min_altitude + constraints_.emergency_stop_dist ||
        state_.position.z > constraints_.max_altitude - constraints_.emergency_stop_dist) {
        return true;
    }
    
    return false;
}

void Drone::emergencyStop() {
    state_.velocity = 0.0f;
    state_.angular_velocity = 0.0f;
    state_.pitch_rate = 0.0f;
    state_.roll_rate = 0.0f;
    state_.vertical_velocity = 0.0f;
}

bool Drone::isWithinConstraints(float throttle, float yaw_rate, float pitch_rate, 
                               float roll_rate, float vertical_thrust) const {
    if (std::abs(throttle) > 1.0f) return false;
    if (std::abs(yaw_rate) > constraints_.max_angular_velocity) return false;
    if (std::abs(pitch_rate) > constraints_.max_pitch_rate) return false;
    if (std::abs(roll_rate) > constraints_.max_roll_rate) return false;
    if (std::abs(vertical_thrust) > 1.0f) return false;
    
    // Check minimum turn radius only when moving forward
    if (throttle > 0.1f && std::abs(yaw_rate) > 0) {
        float turn_radius = state_.velocity / std::abs(yaw_rate);
        if (turn_radius < constraints_.min_turn_radius) return false;
    }
    
    return true;
}

bool Drone::wouldCollide(float throttle, float yaw_rate, float pitch_rate, float roll_rate,
                         float vertical_thrust, float dt) const {
    if (!world_) return false;
    
    // Calculate new position
    cv::Point3f new_position = calculate3DPosition(dt, throttle, yaw_rate, pitch_rate, roll_rate, vertical_thrust);
    
    // Check if new position would collide with obstacles
    if (check3DCollision(new_position, constraints_.safety_margin)) {
        return true;
    }
    
    // Check if new position would be out of bounds
    if (!world_->isInBounds(new_position)) {
        return true;
    }
    
    // Check altitude bounds
    if (new_position.z < constraints_.min_altitude || new_position.z > constraints_.max_altitude) {
        return true;
    }
    
    // Check if new position would be too close to boundaries
    float margin = constraints_.safety_margin;
    auto size = world_->getSize();
    if (new_position.x < margin || new_position.x >= size.width - margin ||
        new_position.y < margin || new_position.y >= size.height - margin ||
        new_position.z < margin || new_position.z >= size.depth - margin) {
        return true;
    }
    
    return false;
}

cv::Mat Drone::getTopDownView(const World& world, int view_size) const {
    cv::Mat view(view_size, view_size, CV_8UC3, cv::Scalar(128, 128, 128));
    
    // Calculate view bounds
    float view_range = max_sensor_range_;
    float half_range = view_range / 2.0f;
    
    // Draw drone position at center
    cv::Point center(view_size / 2, view_size / 2);
    cv::circle(view, center, 5, cv::Scalar(0, 255, 0), -1);
    
    // Draw heading direction
    cv::Point heading_end(center.x + 20 * cos(state_.heading), center.y + 20 * sin(state_.heading));
    cv::arrowedLine(view, center, heading_end, cv::Scalar(0, 255, 0), 2);
    
    // Draw obstacles in view range
    for (const auto& obstacle : world.getObstacles()) {
        cv::Point3f relative_pos = obstacle.position - state_.position;
        if (std::abs(relative_pos.x) < half_range && std::abs(relative_pos.y) < half_range) {
            // Convert to view coordinates
            int view_x = center.x + (relative_pos.x / half_range) * (view_size / 2);
            int view_y = center.y + (relative_pos.y / half_range) * (view_size / 2);
            
            if (view_x >= 0 && view_x < view_size && view_y >= 0 && view_y < view_size) {
                cv::circle(view, cv::Point(view_x, view_y), 
                          std::max(1, int(obstacle.radius / 2)), cv::Scalar(0, 0, 255), -1);
            }
        }
    }
    
    return view;
}

cv::Mat Drone::getEgoView(const World& world, int view_size) const {
    cv::Mat view(view_size, view_size, CV_8UC3, cv::Scalar(128, 128, 128));
    
    // Calculate view bounds
    float view_range = max_sensor_range_;
    float half_range = view_range / 2.0f;
    
    // Draw horizon line
    int horizon_y = view_size / 2;
    cv::line(view, cv::Point(0, horizon_y), cv::Point(view_size, horizon_y), cv::Scalar(255, 255, 255), 1);
    
    // Draw drone position indicator
    cv::Point center(view_size / 2, view_size / 2);
    cv::circle(view, center, 3, cv::Scalar(0, 255, 0), -1);
    
    // Draw obstacles in view range
    for (const auto& obstacle : world.getObstacles()) {
        cv::Point3f relative_pos = obstacle.position - state_.position;
        
        // Check if obstacle is in front of drone
        float forward_distance = relative_pos.x * cos(state_.heading) + relative_pos.y * sin(state_.heading);
        if (forward_distance > 0 && forward_distance < view_range) {
            // Calculate lateral position
            float lateral_distance = -relative_pos.x * sin(state_.heading) + relative_pos.y * cos(state_.heading);
            
            // Convert to view coordinates
            int view_x = center.x + (lateral_distance / half_range) * (view_size / 2);
            int view_y = center.y - (forward_distance / view_range) * (view_size / 2);
            
            if (view_x >= 0 && view_x < view_size && view_y >= 0 && view_y < view_size) {
                cv::circle(view, cv::Point(view_x, view_y), 
                          std::max(1, int(obstacle.radius / 2)), cv::Scalar(0, 0, 255), -1);
            }
        }
    }
    
    return view;
}

cv::Mat Drone::getDepthMap(const World& world, int num_rays) const {
    cv::Mat depth_map(num_rays, 1, CV_32F, cv::Scalar(max_sensor_range_));
    
    float angle_step = 2.0f * M_PI / num_rays;
    
    for (int i = 0; i < num_rays; i++) {
        float angle = i * angle_step;
        float min_distance = max_sensor_range_;
        
        // Cast ray in 3D space
        cv::Point3f ray_direction(cos(angle), sin(angle), 0);
        cv::Point3f ray_end = state_.position + ray_direction * max_sensor_range_;
        
        // Check intersection with obstacles
        for (const auto& obstacle : world.getObstacles()) {
            // Simple sphere-ray intersection
            cv::Point3f to_obstacle = obstacle.position - state_.position;
            float proj_length = to_obstacle.dot(ray_direction);
            
            if (proj_length > 0) {
                cv::Point3f closest_point = state_.position + ray_direction * proj_length;
                float distance_to_ray = cv::norm(obstacle.position - closest_point);
                
                if (distance_to_ray <= obstacle.radius) {
                    float intersection_distance = proj_length - sqrt(obstacle.radius * obstacle.radius - distance_to_ray * distance_to_ray);
                    if (intersection_distance > 0 && intersection_distance < min_distance) {
                        min_distance = intersection_distance;
                    }
                }
            }
        }
        
        depth_map.at<float>(i, 0) = min_distance;
    }
    
    return depth_map;
}

cv::Mat Drone::getOccupancyGrid(const World& world, int grid_size) const {
    cv::Mat grid(grid_size, grid_size, CV_8UC1, cv::Scalar(0));
    
    float cell_size = max_sensor_range_ * 2.0f / grid_size;
    float half_range = max_sensor_range_;
    
    for (int y = 0; y < grid_size; y++) {
        for (int x = 0; x < grid_size; x++) {
            // Convert grid coordinates to world coordinates
            float world_x = state_.position.x + (x - grid_size / 2) * cell_size;
            float world_y = state_.position.y + (y - grid_size / 2) * cell_size;
            cv::Point3f world_pos(world_x, world_y, state_.position.z);
            
            // Check if position is occupied
            if (world.isInBounds(world_pos) && check3DCollision(world_pos, cell_size / 2)) {
                grid.at<uchar>(y, x) = 255;
            }
        }
    }
    
    return grid;
}

cv::Mat Drone::get3DOccupancyGrid(const World& world, int grid_size, int height_layers) const {
    // Create 3D grid as a flattened 2D matrix
    cv::Mat grid(height_layers, grid_size * grid_size, CV_8UC1, cv::Scalar(0));
    
    float cell_size = max_sensor_range_ * 2.0f / grid_size;
    float height_step = (constraints_.max_altitude - constraints_.min_altitude) / height_layers;
    float half_range = max_sensor_range_;
    
    for (int z = 0; z < height_layers; z++) {
        for (int y = 0; y < grid_size; y++) {
            for (int x = 0; x < grid_size; x++) {
                // Convert grid coordinates to world coordinates
                float world_x = state_.position.x + (x - grid_size / 2) * cell_size;
                float world_y = state_.position.y + (y - grid_size / 2) * cell_size;
                float world_z = constraints_.min_altitude + z * height_step;
                cv::Point3f world_pos(world_x, world_y, world_z);
                
                // Check if position is occupied
                if (world.isInBounds(world_pos) && check3DCollision(world_pos, cell_size / 2)) {
                    grid.at<uchar>(z, y * grid_size + x) = 255;
                }
            }
        }
    }
    
    return grid;
}

void Drone::setState(const DroneState& state) {
    state_ = state;
}

void Drone::setConstraints(const DroneConstraints& constraints) {
    constraints_ = constraints;
}

float Drone::getDistanceToGoal(const cv::Point3f& goal) const {
    return cv::norm(state_.position - goal);
}

float Drone::getProgressToGoal(const cv::Point3f& start, const cv::Point3f& goal) const {
    cv::Point3f start_to_goal = goal - start;
    cv::Point3f start_to_current = state_.position - start;
    
    float total_distance = cv::norm(start_to_goal);
    if (total_distance < 1e-6f) return 0.0f;
    
    float progress = start_to_goal.dot(start_to_current) / (total_distance * total_distance);
    return std::max(0.0f, std::min(1.0f, progress));
}

bool Drone::hasReachedGoal(const cv::Point3f& goal, float threshold) const {
    return getDistanceToGoal(goal) < threshold;
}

bool Drone::isInAltitudeBounds() const {
    return state_.position.z >= constraints_.min_altitude && 
           state_.position.z <= constraints_.max_altitude;
}

cv::Point3f Drone::getForwardDirection() const {
    return cv::Point3f(cos(state_.heading) * cos(state_.pitch),
                       sin(state_.heading) * cos(state_.pitch),
                       sin(state_.pitch));
}

cv::Point3f Drone::getUpDirection() const {
    // Calculate up direction based on roll and pitch
    return cv::Point3f(-sin(state_.roll) * sin(state_.pitch),
                       cos(state_.roll) * sin(state_.pitch),
                       cos(state_.pitch));
}

cv::Mat Drone::renderRaycast(const World& world, float start_angle, float end_angle, int num_rays) const {
    cv::Mat raycast(num_rays, 1, CV_32F, cv::Scalar(max_sensor_range_));
    
    float angle_step = (end_angle - start_angle) / (num_rays - 1);
    
    for (int i = 0; i < num_rays; i++) {
        float angle = start_angle + i * angle_step;
        float min_distance = max_sensor_range_;
        
        // Cast ray in 3D space
        cv::Point3f ray_direction(cos(angle), sin(angle), 0);
        cv::Point3f ray_end = state_.position + ray_direction * max_sensor_range_;
        
        // Check intersection with obstacles
        for (const auto& obstacle : world.getObstacles()) {
            // Simple sphere-ray intersection
            cv::Point3f to_obstacle = obstacle.position - state_.position;
            float proj_length = to_obstacle.dot(ray_direction);
            
            if (proj_length > 0) {
                cv::Point3f closest_point = state_.position + ray_direction * proj_length;
                float distance_to_ray = cv::norm(obstacle.position - closest_point);
                
                if (distance_to_ray <= obstacle.radius) {
                    float intersection_distance = proj_length - sqrt(obstacle.radius * obstacle.radius - distance_to_ray * distance_to_ray);
                    if (intersection_distance > 0 && intersection_distance < min_distance) {
                        min_distance = intersection_distance;
                    }
                }
            }
        }
        
        raycast.at<float>(i, 0) = min_distance;
    }
    
    return raycast;
}

cv::Point3f Drone::worldToCamera(const cv::Point3f& world_point) const {
    // Transform world point to camera coordinates
    cv::Point3f relative = world_point - state_.position;
    
    // Apply rotation transformations (simplified - could be made more accurate with rotation matrices)
    float cos_h = cos(-state_.heading);
    float sin_h = sin(-state_.heading);
    float cos_p = cos(-state_.pitch);
    float sin_p = sin(-state_.pitch);
    float cos_r = cos(-state_.roll);
    float sin_r = sin(-state_.roll);
    
    // Apply yaw rotation
    float x1 = relative.x * cos_h - relative.y * sin_h;
    float y1 = relative.x * sin_h + relative.y * cos_h;
    float z1 = relative.z;
    
    // Apply pitch rotation
    float x2 = x1 * cos_p + z1 * sin_p;
    float y2 = y1;
    float z2 = -x1 * sin_p + z1 * cos_p;
    
    // Apply roll rotation
    float x3 = x2;
    float y3 = y2 * cos_r - z2 * sin_r;
    float z3 = y2 * sin_r + z2 * cos_r;
    
    return cv::Point3f(x3, y3, z3);
}

cv::Point3f Drone::cameraToWorld(const cv::Point3f& camera_point) const {
    // Transform camera point back to world coordinates
    // Apply inverse rotation transformations
    float cos_h = cos(state_.heading);
    float sin_h = sin(state_.heading);
    float cos_p = cos(state_.pitch);
    float sin_p = sin(state_.pitch);
    float cos_r = cos(state_.roll);
    float sin_r = sin(state_.roll);
    
    // Apply inverse roll rotation
    float x1 = camera_point.x;
    float y1 = camera_point.y * cos_r + camera_point.z * sin_r;
    float z1 = -camera_point.y * sin_r + camera_point.z * cos_r;
    
    // Apply inverse pitch rotation
    float x2 = x1 * cos_p - z1 * sin_p;
    float y2 = y1;
    float z2 = x1 * sin_p + z1 * cos_p;
    
    // Apply inverse yaw rotation
    float x3 = x2 * cos_h + y2 * sin_h;
    float y3 = -x2 * sin_h + y2 * cos_h;
    float z3 = z2;
    
    return state_.position + cv::Point3f(x3, y3, z3);
}

} // namespace sim
