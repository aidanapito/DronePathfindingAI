#pragma once

#include <opencv2/opencv.hpp>
#include <chrono>

class Drone {
public:
    struct State {
        cv::Point3f position;      // 3D position (x, y, z)
        cv::Point3f velocity;      // 3D velocity
        cv::Point3f orientation;   // Euler angles (roll, pitch, yaw) in radians
        float altitude;            // Current altitude above ground
    };

    Drone(const cv::Point3f& start_pos = cv::Point3f(0, 0, 100));
    
    // Update drone physics
    void update(float delta_time, float throttle, float yaw_rate, 
                float pitch_rate = 0.0f, float roll_rate = 0.0f, 
                float vertical_thrust = 0.0f);
    
    // Getters
    const State& getState() const { return state_; }
    cv::Point3f getPosition() const { return state_.position; }
    cv::Point3f getOrientation() const { return state_.orientation; }
    float getAltitude() const { return state_.altitude; }
    
    // Physics constants
    static constexpr float MAX_SPEED = 150.0f;        // Increased from 100.0f
    static constexpr float MAX_ALTITUDE = 200.0f;       // Max flying height
    static constexpr float MIN_ALTITUDE = 10.0f;        // Min flying height
    static constexpr float TURN_RATE = 6.0f;          // Increased from 4.0f
    static constexpr float ACCELERATION = 75.0f;      // Increased from 50.0f
    static constexpr float DRAG = 0.98f;              // Keep same for smooth deceleration

private:
    State state_;
    cv::Point3f acceleration_;
    
    // Helper methods
    void applyPhysics(float delta_time);
    void constrainMovement();
    cv::Point3f rotateVector(const cv::Point3f& vec, const cv::Point3f& angles);
};
