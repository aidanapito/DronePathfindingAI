#pragma once

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <vector>

// Forward declaration
namespace sim {
    class World;
}

namespace sim {

struct DroneState {
    cv::Point2f position;      // x, y in world coordinates
    float heading;              // heading angle in radians
    float velocity;             // current velocity magnitude
    float angular_velocity;     // angular velocity in rad/s
};

struct DroneConstraints {
    float max_velocity;         // maximum forward velocity
    float max_angular_velocity; // maximum turn rate
    float min_turn_radius;      // minimum turning radius
    float safety_margin;        // safety distance from obstacles
    float emergency_stop_dist;  // distance to trigger emergency stop
};

class Drone {
public:
    Drone(const cv::Point2f& start_pos, float start_heading = 0.0f);
    ~Drone() = default;

    // Kinematics update
    void update(float dt, float throttle, float yaw_rate);
    
    // Safety layer
    bool isEmergencyStop() const;
    void emergencyStop();
    bool isWithinConstraints(float throttle, float yaw_rate) const;
    
    // Sensor simulation
    cv::Mat getTopDownView(const World& world, int view_size = 200) const;
    cv::Mat getEgoView(const World& world, int view_size = 84) const;
    cv::Mat getDepthMap(const World& world, int num_rays = 36) const;
    cv::Mat getOccupancyGrid(const World& world, int grid_size = 21) const;
    
    // State management
    void setState(const DroneState& state);
    const DroneState& getState() const { return state_; }
    
    // Constraints
    void setConstraints(const DroneConstraints& constraints);
    const DroneConstraints& getConstraints() const { return constraints_; }
    
    // World reference
    void setWorld(const World* world) { world_ = world; }
    
    // Utility
    float getDistanceToGoal(const cv::Point2f& goal) const;
    float getProgressToGoal(const cv::Point2f& start, const cv::Point2f& goal) const;
    bool hasReachedGoal(const cv::Point2f& goal, float threshold = 10.0f) const;

private:
    DroneState state_;
    DroneConstraints constraints_;
    const World* world_;  // Reference to world for collision detection
    
    // Sensor parameters
    float camera_height_;
    float camera_fov_;
    float max_sensor_range_;
    
    // Helper methods
    cv::Mat renderRaycast(const World& world, float start_angle, float end_angle, int num_rays) const;
    cv::Point2f worldToCamera(const cv::Point2f& world_point) const;
    cv::Point2f cameraToWorld(const cv::Point2f& camera_point) const;
};

} // namespace sim
