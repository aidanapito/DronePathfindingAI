#pragma once

#include <cmath>

struct DroneState {
    float x, y, z;           // Position
    float vx, vy, vz;        // Velocity
    float roll, pitch, yaw;  // Orientation (radians)
};

struct DroneInput {
    float forward_thrust;    // Forward/backward thrust
    float yaw_rate;          // Yaw rotation rate
    float pitch_rate;        // Pitch rotation rate
    float roll_rate;         // Roll rotation rate
    float vertical_thrust;   // Up/down thrust
};

class Drone {
public:
    Drone();
    
    // Update drone physics
    void update(float delta_time, const DroneInput& input);
    
    // Getters
    DroneState getPosition() const;
    DroneState getOrientation() const;
    
    // Physics constants
    static constexpr float MAX_SPEED = 150.0f;
    static constexpr float MAX_ALTITUDE = 200.0f;
    static constexpr float MIN_ALTITUDE = 10.0f;
    static constexpr float TURN_RATE = 6.0f;
    static constexpr float ACCELERATION = 75.0f;
    static constexpr float DRAG = 0.98f;

private:
    DroneState state_;
    
    // Helper methods
    void applyPhysics(float delta_time);
    void constrainMovement();
};
