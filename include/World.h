#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <random>

struct Obstacle {
    cv::Point3f position;
    float radius;
    float height;
    bool is_vertical;  // true for buildings, false for ground obstacles
    cv::Scalar color;
};

class World {
public:
    World(int width = 1200, int height = 800, int depth = 600);
    
    // World generation
    void generateTerrain();
    void addSkyscrapers(int count);
    void addGroundObstacles(int count);
    void addMountainPass();
    
    // Rendering
    void render3D(cv::Mat& output, const cv::Point3f& camera_pos, 
                  const cv::Point3f& camera_target) const;
    
    // Getters
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    int getDepth() const { return depth_; }
    const std::vector<Obstacle>& getObstacles() const { return obstacles_; }
    
    // Collision detection
    bool checkCollision(const cv::Point3f& position, float radius) const;
    float getGroundHeight(float x, float y) const;
    
    // Rendering helpers
    cv::Point2f project3DTo2D(const cv::Point3f& point3d, 
                               const cv::Point3f& camera_pos,
                               const cv::Point3f& camera_target) const;

private:
    int width_, height_, depth_;
    std::vector<Obstacle> obstacles_;
    std::mt19937 rng_;
    
    // Terrain generation helpers
    void generateSkyscraper(float x, float y);
    void generateGroundObstacle(float x, float y);
    void generateMountain(float x, float y, float height);
};
