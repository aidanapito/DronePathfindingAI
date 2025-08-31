#pragma once

#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>
#include <string>

class Renderer {
public:
    Renderer(int width, int height);
    ~Renderer();
    
    // Window management
    bool initialize();
    void shutdown();
    bool shouldClose() const;
    void swapBuffers();
    void pollEvents();
    
    // Rendering
    void beginFrame();
    void endFrame();
    void clear(const glm::vec3& color = glm::vec3(0.1f, 0.1f, 0.1f));
    
    // 3D rendering
    void render3DScene(const glm::vec3& cameraPos, const glm::vec3& cameraTarget, 
                      const glm::vec3& cameraUp, const std::vector<struct Obstacle>& obstacles);
    
    // Drone rendering
    void renderXFrameDrone(const glm::vec3& position, const glm::vec3& orientation, const glm::vec3& color, bool isThirdPerson = false);
    void renderCrashMessage();
    void renderText(const std::string& text, float x, float y, float scale, const glm::vec3& color);
    
    // Camera setup
    void setProjectionMatrix(float fov, float aspect, float near, float far);
    void setViewMatrix(const glm::vec3& position, const glm::vec3& target, const glm::vec3& up);
    
    // Getters
    GLFWwindow* getWindow() const { return window_; }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    
private:
    // OpenGL setup
    bool setupShaders();
    bool setupBuffers();
    void setup3DObjects();
    
    // Rendering helpers
    void renderCube(const glm::vec3& position, const glm::vec3& size, const glm::vec3& color);
    void renderPyramid(const glm::vec3& position, const glm::vec3& size, const glm::vec3& color);
    void renderCylinder(const glm::vec3& position, float radius, float height, const glm::vec3& color);
    void renderSphere(const glm::vec3& position, float radius, const glm::vec3& color);
    void renderGround();
    void renderSkybox();
    
    // Shader management
    unsigned int createShader(const std::string& vertexSource, const std::string& fragmentSource);
    unsigned int compileShader(unsigned int type, const std::string& source);
    
private:
    GLFWwindow* window_;
    int width_, height_;
    
    // Shaders
    unsigned int shaderProgram_;
    unsigned int vertexShader_;
    unsigned int fragmentShader_;
    
    // Matrices
    glm::mat4 projectionMatrix_;
    glm::mat4 viewMatrix_;
    
    // Buffers
    unsigned int cubeVAO_, cubeVBO_, cubeEBO_;
    unsigned int groundVAO_, groundVBO_;
    
    // Shader locations
    int modelLoc_, viewLoc_, projectionLoc_, colorLoc_;
};
