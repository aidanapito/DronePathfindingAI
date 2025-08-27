#pragma once

#include "../sim/World.h"
#include "../sim/Drone.h"
#include "../bridge/Environment.h"
#include <opencv2/opencv.hpp>
#include <memory>

namespace ui {

struct Camera3D {
    cv::Point3f position;      // Camera position in 3D space
    cv::Point3f target;        // Point the camera is looking at
    cv::Point3f up;            // Up vector for camera orientation
    float fov;                 // Field of view in degrees
    float near_plane;          // Near clipping plane
    float far_plane;           // Far clipping plane
    
    Camera3D() : position(0, 0, 300), target(0, 0, 0), up(0, 1, 0), 
                  fov(60.0f), near_plane(1.0f), far_plane(1000.0f) {}
};

class SimulatorUI {
public:
    SimulatorUI();
    ~SimulatorUI() = default;

    // 2D rendering (backward compatibility)
    void render2D(const sim::World& world, const sim::Drone& drone, 
                  const std::vector<cv::Point2f>& path = {});
    
    // 3D rendering
    void render3D(const sim::World& world, const sim::Drone& drone, 
                  const std::vector<cv::Point3f>& path_3d = {},
                  const std::vector<cv::Point2f>& path_2d = {});
    
    // Camera controls
    void setCamera(const Camera3D& camera) { camera_3d_ = camera; }
    void orbitCamera(float delta_azimuth, float delta_elevation);
    void zoomCamera(float delta_distance);
    void panCamera(float delta_x, float delta_y);
    void resetCamera();
    
    // Display options
    void setShowGrid(bool show) { show_grid_ = show; }
    void setShowAxes(bool show) { show_axes_ = show; }
    void setShowPath(bool show) { show_path_ = show; }
    void setShowObstacles(bool show) { show_obstacles_ = show; }
    void setShowDrone(bool show) { show_drone_ = show; }
    void setShowGoal(bool show) { show_goal_ = show; }
    
    // Rendering modes
    void setRenderMode(const std::string& mode) { render_mode_ = mode; }
    std::string getRenderMode() const { return render_mode_; }
    
    // Window management
    void createWindow(const std::string& name, int width = 1200, int height = 800);
    void updateWindow();
    void closeWindow();
    
    // Event handling
    void handleMouseEvents();
    void handleKeyboardEvents();
    
    // Utility
    cv::Mat getCurrentFrame() const { return current_frame_; }
    void saveFrame(const std::string& filename) const;

private:
    // 3D rendering helpers
    void render3DGrid();
    void render3DAxes();
    void render3DObstacles(const sim::World& world);
    void render3DDrone(const sim::Drone& drone);
    void render3DPath(const std::vector<cv::Point3f>& path_3d, 
                      const std::vector<cv::Point2f>& path_2d);
    void render3DGoal(const sim::World& world);
    void renderSpeedIndicator(const sim::Drone& drone);
    
    // 3D math helpers
    cv::Point2f project3DTo2D(const cv::Point3f& point_3d) const;
    cv::Point3f screenTo3D(const cv::Point2f& screen_point, float depth) const;
    cv::Mat getViewMatrix() const;
    cv::Mat getProjectionMatrix() const;
    
    // 2D rendering helpers (backward compatibility)
    void render2DGrid();
    void render2DObstacles(const sim::World& world);
    void render2DDrone(const sim::Drone& drone);
    void render2DPath(const std::vector<cv::Point2f>& path);
    void render2DGoal(const sim::World& world);
    
    // UI state
    cv::Mat current_frame_;
    std::string window_name_;
    bool window_created_;
    
    // 3D camera
    Camera3D camera_3d_;
    
    // Display options
    bool show_grid_;
    bool show_axes_;
    bool show_path_;
    bool show_obstacles_;
    bool show_drone_;
    bool show_goal_;
    
    // Rendering mode
    std::string render_mode_; // "2D", "3D", "TopDown", "FirstPerson"
    
    // Mouse interaction
    cv::Point2f last_mouse_pos_;
    bool mouse_dragging_;
    bool mouse_right_dragging_;
    
    // Colors
    cv::Scalar grid_color_;
    cv::Scalar axes_color_;
    cv::Scalar path_color_;
    cv::Scalar drone_color_;
    cv::Scalar goal_color_;
    cv::Scalar obstacle_color_;
    cv::Scalar background_color_;
    
    // Constants
    static constexpr float GRID_SIZE = 50.0f;
    static constexpr float AXIS_LENGTH = 100.0f;
    static constexpr float DRONE_SIZE = 15.0f;
    static constexpr float GOAL_SIZE = 20.0f;
};

} // namespace ui
