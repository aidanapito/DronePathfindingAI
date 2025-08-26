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
    cv::Point3f position;      // x, y, z in world coordinates
    float heading;              // yaw angle in radians (rotation around z-axis)
    float pitch;                // pitch angle in radians (rotation around x-axis)
    float roll;                 // roll angle in radians (rotation around y-axis)
    float velocity;             // current forward velocity magnitude
    float angular_velocity;     // yaw angular velocity in rad/s
    float pitch_rate;           // pitch angular velocity in rad/s
    float roll_rate;            // roll angular velocity in rad/s
    float vertical_velocity;    // vertical velocity (positive = up)
};

struct DroneConstraints {
    float max_velocity;         // maximum forward velocity
    float max_angular_velocity; // maximum yaw turn rate
    float max_pitch_rate;       // maximum pitch rate
    float max_roll_rate;        // maximum roll rate
    float max_vertical_velocity; // maximum vertical velocity
    float min_turn_radius;      // minimum turning radius
    float safety_margin;        // safety distance from obstacles
    float emergency_stop_dist;  // distance to trigger emergency stop
    float max_altitude;         // maximum allowed altitude
    float min_altitude;         // minimum allowed altitude (ground level)
};

class Drone {
public:
    Drone(const cv::Point3f& start_pos, float start_heading = 0.0f, 
          float start_pitch = 0.0f, float start_roll = 0.0f);
    ~Drone() = default;

    // Kinematics update
    void update(float dt, float throttle, float yaw_rate, float pitch_rate = 0.0f, 
                float roll_rate = 0.0f, float vertical_thrust = 0.0f);
    
    // Safety layer
    bool isEmergencyStop() const;
    void emergencyStop();
    bool isWithinConstraints(float throttle, float yaw_rate, float pitch_rate = 0.0f,
                           float roll_rate = 0.0f, float vertical_thrust = 0.0f) const;
    bool wouldCollide(float throttle, float yaw_rate, float pitch_rate, float roll_rate,
                     float vertical_thrust, float dt) const;
    
    // Sensor simulation
    cv::Mat getTopDownView(const World& world, int view_size = 200) const;
    cv::Mat getEgoView(const World& world, int view_size = 84) const;
    cv::Mat getDepthMap(const World& world, int num_rays = 36) const;
    cv::Mat getOccupancyGrid(const World& world, int grid_size = 21) const;
    cv::Mat get3DOccupancyGrid(const World& world, int grid_size = 21, int height_layers = 10) const;
    
    // State management
    void setState(const DroneState& state);
    const DroneState& getState() const { return state_; }
    
    // Constraints
    void setConstraints(const DroneConstraints& constraints);
    const DroneConstraints& getConstraints() const { return constraints_; }
    
    // World reference
    void setWorld(const World* world) { world_ = world; }
    
    // Utility
    float getDistanceToGoal(const cv::Point3f& goal) const;
    float getProgressToGoal(const cv::Point3f& start, const cv::Point3f& goal) const;
    bool hasReachedGoal(const cv::Point3f& goal, float threshold = 10.0f) const;
    
    // 3D specific methods
    float getAltitude() const { return state_.position.z; }
    bool isInAltitudeBounds() const;
    cv::Point3f getForwardDirection() const;
    cv::Point3f getUpDirection() const;

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
    cv::Point3f worldToCamera(const cv::Point3f& world_point) const;
    cv::Point3f cameraToWorld(const cv::Point3f& camera_point) const;
    
    // 3D physics helpers
    void update3DKinematics(float dt, float throttle, float yaw_rate, float pitch_rate, 
                           float roll_rate, float vertical_thrust);
    cv::Point3f calculate3DPosition(float dt, float throttle, float yaw_rate, 
                                   float pitch_rate, float roll_rate, float vertical_thrust) const;
    bool check3DCollision(const cv::Point3f& position, float radius) const;
};

} // namespace sim
