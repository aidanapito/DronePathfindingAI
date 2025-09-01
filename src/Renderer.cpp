#include "Renderer.h"
#include "World.h"
#include <iostream>
#include <fstream>
#include <sstream>

// OpenGL extension function types for VAOs (needed on macOS)
typedef void (*PFNGLGENVERTEXARRAYSPROC)(GLsizei n, GLuint* arrays);
typedef void (*PFNGLBINDVERTEXARRAYPROC)(GLuint array);
typedef void (*PFNGLDELETEVERTEXARRAYSPROC)(GLsizei n, const GLuint* arrays);

// OpenGL extension function pointers for VAOs (needed on macOS)
PFNGLGENVERTEXARRAYSPROC glGenVertexArrays;
PFNGLBINDVERTEXARRAYPROC glBindVertexArray;
PFNGLDELETEVERTEXARRAYSPROC glDeleteVertexArrays;

Renderer::Renderer(int width, int height) : width_(width), height_(height), window_(nullptr) {
    // Initialize OpenGL context
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return;
    }
    
    // Configure GLFW
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    
    // Create window
    window_ = glfwCreateWindow(width_, height_, "3D Drone Simulator", nullptr, nullptr);
    if (!window_) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return;
    }
    
    glfwMakeContextCurrent(window_);
    
    // Load OpenGL extension functions
    glGenVertexArrays = (PFNGLGENVERTEXARRAYSPROC)glfwGetProcAddress("glGenVertexArrays");
    glBindVertexArray = (PFNGLBINDVERTEXARRAYPROC)glfwGetProcAddress("glBindVertexArray");
    glDeleteVertexArrays = (PFNGLDELETEVERTEXARRAYSPROC)glfwGetProcAddress("glDeleteVertexArrays");
    
    if (!glGenVertexArrays || !glBindVertexArray || !glDeleteVertexArrays) {
        std::cerr << "Failed to load OpenGL extension functions" << std::endl;
        return;
    }
    
    // Initialize OpenGL
    if (!setupShaders()) {
        std::cerr << "Failed to setup shaders" << std::endl;
        return;
    }
    
    if (!setupBuffers()) {
        std::cerr << "Failed to setup buffers" << std::endl;
        return;
    }
    
    setup3DObjects();
    
    // Enable depth testing
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    
    // Set projection matrix
    setProjectionMatrix(45.0f, (float)width_ / (float)height_, 0.1f, 1000.0f);
}

Renderer::~Renderer() {
    shutdown();
}

bool Renderer::initialize() {
    return window_ != nullptr;
}

void Renderer::shutdown() {
    if (window_) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
    }
    glfwTerminate();
}

bool Renderer::shouldClose() const {
    return glfwWindowShouldClose(window_);
}

void Renderer::swapBuffers() {
    glfwSwapBuffers(window_);
}

void Renderer::pollEvents() {
    glfwPollEvents();
}

void Renderer::beginFrame() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::endFrame() {
    // Frame is complete
}

void Renderer::clear(const glm::vec3& color) {
    glClearColor(color.r, color.g, color.b, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::setProjectionMatrix(float fov, float aspect, float near, float far) {
    projectionMatrix_ = glm::perspective(glm::radians(fov), aspect, near, far);
}

void Renderer::setViewMatrix(const glm::vec3& position, const glm::vec3& target, const glm::vec3& up) {
    viewMatrix_ = glm::lookAt(position, target, up);
}

void Renderer::render3DScene(const glm::vec3& cameraPos, const glm::vec3& cameraTarget, 
                           const glm::vec3& cameraUp, const std::vector<struct Obstacle>& obstacles) {
    // Set view matrix
    setViewMatrix(cameraPos, cameraTarget, cameraUp);
    
    // Use shader program
    glUseProgram(shaderProgram_);
    
    // Set uniforms
    glUniformMatrix4fv(projectionLoc_, 1, GL_FALSE, glm::value_ptr(projectionMatrix_));
    glUniformMatrix4fv(viewLoc_, 1, GL_FALSE, glm::value_ptr(viewMatrix_));
    glUniform3fv(glGetUniformLocation(shaderProgram_, "lightPos"), 1, glm::value_ptr(glm::vec3(100.0f, 100.0f, 200.0f)));
    glUniform3fv(glGetUniformLocation(shaderProgram_, "viewPos"), 1, glm::value_ptr(cameraPos));
    
    // Render ground
    renderGround();
    
    // Render obstacles
    for (const auto& obstacle : obstacles) {
        glm::vec3 color;
        glm::vec3 size;
        // Transform obstacle coordinates to match new coordinate system
        // Original: X=left/right, Y=forward/backward, Z=up/down
        // New: X=left/right, Y=up/down, Z=forward/backward
        glm::vec3 position(obstacle.x, obstacle.z, obstacle.y);
        
        switch (obstacle.type) {
            case ObstacleType::SKYSCRAPER:
                // Vary skyscraper colors based on height
                if (obstacle.height > 150.0f) {
                    color = glm::vec3(0.8f, 0.8f, 0.9f); // Light blue-gray for tall buildings
                } else if (obstacle.height > 100.0f) {
                    color = glm::vec3(0.7f, 0.7f, 0.8f); // Medium blue-gray
                } else {
                    color = glm::vec3(0.6f, 0.6f, 0.6f); // Gray
                }
                // Use actual obstacle dimensions and make it vertical (height in Y direction)
                size = glm::vec3(obstacle.width, obstacle.height, obstacle.depth);
                break;
            case ObstacleType::GROUND_OBSTACLE:
                color = glm::vec3(0.55f, 0.27f, 0.07f); // Brown
                size = glm::vec3(30.0f, 30.0f, 20.0f);
                break;
            case ObstacleType::MOUNTAIN:
                color = glm::vec3(0.4f, 0.4f, 0.4f); // Dark gray
                size = glm::vec3(80.0f, 80.0f, 60.0f);
                break;
            case ObstacleType::BRIDGE:
                color = glm::vec3(0.3f, 0.3f, 0.3f); // Dark gray
                size = glm::vec3(obstacle.width, obstacle.height, obstacle.depth);
                break;
            case ObstacleType::TUNNEL:
                color = glm::vec3(0.2f, 0.2f, 0.2f); // Very dark gray
                size = glm::vec3(obstacle.width, obstacle.height, obstacle.depth);
                break;
            case ObstacleType::ARCH:
                color = glm::vec3(0.7f, 0.7f, 0.7f); // Light gray
                size = glm::vec3(obstacle.width, obstacle.height, obstacle.depth);
                break;
            case ObstacleType::PYRAMID:
                color = glm::vec3(0.8f, 0.6f, 0.2f); // Gold/sand
                size = glm::vec3(obstacle.width, obstacle.height, obstacle.depth);
                break;
            case ObstacleType::SPHERE_BUILDING:
                // Vary sphere building colors
                if (obstacle.height > 100.0f) {
                    color = glm::vec3(0.1f, 0.4f, 0.7f); // Dark blue for tall spheres
                } else {
                    color = glm::vec3(0.2f, 0.6f, 0.8f); // Blue
                }
                size = glm::vec3(obstacle.width, obstacle.height, obstacle.depth);
                break;
            case ObstacleType::WIND_TURBINE:
                color = glm::vec3(0.9f, 0.9f, 0.9f); // White
                size = glm::vec3(obstacle.width, obstacle.height, obstacle.depth);
                break;
            case ObstacleType::RADIO_TOWER:
                color = glm::vec3(0.1f, 0.1f, 0.1f); // Very dark
                size = glm::vec3(obstacle.width, obstacle.height, obstacle.depth);
                break;
            case ObstacleType::WATER_TOWER:
                color = glm::vec3(0.8f, 0.8f, 0.8f); // Light gray
                size = glm::vec3(obstacle.width, obstacle.height, obstacle.depth);
                break;
            case ObstacleType::FACTORY:
                color = glm::vec3(0.5f, 0.3f, 0.1f); // Brown/industrial
                size = glm::vec3(obstacle.width, obstacle.height, obstacle.depth);
                break;
            default:
                color = glm::vec3(1.0f, 1.0f, 1.0f); // White
                size = glm::vec3(20.0f, 20.0f, 20.0f);
                break;
        }
        
        // Apply rotation for directional objects and use appropriate rendering method
        if (obstacle.rotation != 0.0f) {
            glm::mat4 model = glm::mat4(1.0f);
            model = glm::translate(model, position);
            model = glm::rotate(model, obstacle.rotation, glm::vec3(0.0f, 1.0f, 0.0f));
            model = glm::scale(model, size);
            glUniformMatrix4fv(modelLoc_, 1, GL_FALSE, glm::value_ptr(model));
            glUniform3fv(colorLoc_, 1, glm::value_ptr(color));
            glDrawArrays(GL_TRIANGLES, 0, 36); // Draw cube
        } else {
            // Use appropriate rendering method based on obstacle type
            switch (obstacle.type) {
                case ObstacleType::PYRAMID:
                    renderPyramid(position, size, color);
                    break;
                case ObstacleType::SPHERE_BUILDING:
                    renderSphere(position, size.x / 2.0f, color); // radius = width/2
                    break;
                case ObstacleType::WIND_TURBINE:
                case ObstacleType::RADIO_TOWER:
                case ObstacleType::WATER_TOWER:
                    renderCylinder(position, obstacle.radius, obstacle.height, color);
                    break;
                default:
                    renderCube(position, size, color);
                    break;
            }
        }
    }
    
    // Render the drone (H-frame design)
    // Note: This will be called from main.cpp with actual drone position and orientation
}

bool Renderer::setupShaders() {
    // Read shader files
    std::string vertexCode, fragmentCode;
    std::ifstream vShaderFile, fShaderFile;
    
    vShaderFile.open("../shaders/vertex.glsl");
    fShaderFile.open("../shaders/fragment.glsl");
    
    if (!vShaderFile.is_open() || !fShaderFile.is_open()) {
        std::cerr << "Failed to open shader files" << std::endl;
        return false;
    }
    
    std::stringstream vShaderStream, fShaderStream;
    vShaderStream << vShaderFile.rdbuf();
    fShaderStream << fShaderFile.rdbuf();
    
    vShaderFile.close();
    fShaderFile.close();
    
    vertexCode = vShaderStream.str();
    fragmentCode = fShaderStream.str();
    
    // Create shaders
    shaderProgram_ = createShader(vertexCode, fragmentCode);
    if (shaderProgram_ == 0) return false;
    
    // Get uniform locations
    modelLoc_ = glGetUniformLocation(shaderProgram_, "model");
    viewLoc_ = glGetUniformLocation(shaderProgram_, "view");
    projectionLoc_ = glGetUniformLocation(shaderProgram_, "projection");
    colorLoc_ = glGetUniformLocation(shaderProgram_, "objectColor");
    
    return true;
}

bool Renderer::setupBuffers() {
    // Cube vertices with normals
    float vertices[] = {
        // positions          // normals
        -0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
         0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
         0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
         0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
        -0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
        -0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
        
        -0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
         0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
         0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
         0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
        -0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
        -0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
        
        -0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,
        -0.5f,  0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
        -0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
        -0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
        -0.5f, -0.5f,  0.5f, -1.0f,  0.0f,  0.0f,
        -0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,
        
         0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,
         0.5f,  0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
         0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
         0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
         0.5f, -0.5f,  0.5f,  1.0f,  0.0f,  0.0f,
         0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,
        
        -0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,
         0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,
         0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
         0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
        -0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
        -0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,
        
        -0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,
         0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,
         0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
         0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
        -0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
        -0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f
    };
    
    // Create cube VAO and VBO
    glGenVertexArrays(1, &cubeVAO_);
    glGenBuffers(1, &cubeVBO_);
    
    glBindVertexArray(cubeVAO_);
    glBindBuffer(GL_ARRAY_BUFFER, cubeVBO_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
    
    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    
    // Normal attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    
    // Ground vertices - transformed to match new coordinate system
    // Original: X=left/right, Y=forward/backward, Z=up/down
    // New: X=left/right, Y=up/down, Z=forward/backward
    // Using two triangles instead of triangle fan to avoid rendering issues
    float groundVertices[] = {
        // First triangle
        -1000.0f, 0.0f, -1000.0f,  // X=left, Y=ground level, Z=backward
         1000.0f, 0.0f, -1000.0f,  // X=right, Y=ground level, Z=backward
         1000.0f, 0.0f,  1000.0f,  // X=right, Y=ground level, Z=forward
        // Second triangle
        -1000.0f, 0.0f, -1000.0f,  // X=left, Y=ground level, Z=backward
         1000.0f, 0.0f,  1000.0f,  // X=right, Y=ground level, Z=forward
        -1000.0f, 0.0f,  1000.0f   // X=left, Y=ground level, Z=forward
    };
    
    glGenVertexArrays(1, &groundVAO_);
    glGenBuffers(1, &groundVBO_);
    
    glBindVertexArray(groundVAO_);
    glBindBuffer(GL_ARRAY_BUFFER, groundVBO_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(groundVertices), groundVertices, GL_STATIC_DRAW);
    
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    
    return true;
}

void Renderer::setup3DObjects() {
    // Additional setup if needed
}

void Renderer::renderCube(const glm::vec3& position, const glm::vec3& size, const glm::vec3& color) {
    glBindVertexArray(cubeVAO_);
    
    // Set color uniform
    glUniform3fv(colorLoc_, 1, glm::value_ptr(color));
    
    // Create model matrix
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, position);
    model = glm::scale(model, size);
    
    // Set model uniform
    glUniformMatrix4fv(modelLoc_, 1, GL_FALSE, glm::value_ptr(model));
    
    // Draw cube
    glDrawArrays(GL_TRIANGLES, 0, 36);
}

void Renderer::renderPyramid(const glm::vec3& position, const glm::vec3& size, const glm::vec3& color) {
    glBindVertexArray(cubeVAO_);
    
    // Set color uniform
    glUniform3fv(colorLoc_, 1, glm::value_ptr(color));
    
    // Create model matrix for pyramid (scaled cube that gets smaller at top)
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, position);
    model = glm::scale(model, size);
    
    // Set model uniform
    glUniformMatrix4fv(modelLoc_, 1, GL_FALSE, glm::value_ptr(model));
    
    // Draw pyramid (using cube for now - could be enhanced with custom geometry)
    glDrawArrays(GL_TRIANGLES, 0, 36);
}

void Renderer::renderCylinder(const glm::vec3& position, float radius, float height, const glm::vec3& color) {
    glBindVertexArray(cubeVAO_);
    
    // Set color uniform
    glUniform3fv(colorLoc_, 1, glm::value_ptr(color));
    
    // Create model matrix for cylinder (scaled cube)
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, position);
    model = glm::scale(model, glm::vec3(radius * 2.0f, height, radius * 2.0f));
    
    // Set model uniform
    glUniformMatrix4fv(modelLoc_, 1, GL_FALSE, glm::value_ptr(model));
    
    // Draw cylinder (using cube for now - could be enhanced with custom geometry)
    glDrawArrays(GL_TRIANGLES, 0, 36);
}

void Renderer::renderSphere(const glm::vec3& position, float radius, const glm::vec3& color) {
    glBindVertexArray(cubeVAO_);
    
    // Set color uniform
    glUniform3fv(colorLoc_, 1, glm::value_ptr(color));
    
    // Create model matrix for sphere (scaled cube)
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, position);
    model = glm::scale(model, glm::vec3(radius * 2.0f, radius * 2.0f, radius * 2.0f));
    
    // Set model uniform
    glUniformMatrix4fv(modelLoc_, 1, GL_FALSE, glm::value_ptr(model));
    
    // Draw sphere (using cube for now - could be enhanced with custom geometry)
    glDrawArrays(GL_TRIANGLES, 0, 36);
}

void Renderer::renderGround() {
    glBindVertexArray(groundVAO_);
    
    // Set ground color
    glUniform3fv(colorLoc_, 1, glm::value_ptr(glm::vec3(0.2f, 0.8f, 0.2f)));
    
    // Create ground model matrix - position slightly below 0 to avoid z-fighting
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(0.0f, -0.1f, 0.0f));
    glUniformMatrix4fv(modelLoc_, 1, GL_FALSE, glm::value_ptr(model));
    
    // Draw ground as two triangles instead of triangle fan
    glDrawArrays(GL_TRIANGLES, 0, 6);
}

void Renderer::renderXFrameDrone(const glm::vec3& position, const glm::vec3& orientation, const glm::vec3& color, bool isThirdPerson) {
    // Render the drone with proper roll, pitch, and yaw rotations
    
    glm::mat4 droneModel = glm::mat4(1.0f);
    droneModel = glm::translate(droneModel, position);
    
    // Apply rotations in the correct order: roll, pitch, yaw
    // This ensures roll is applied in the drone's local coordinate system
    // Roll (around Z-axis in OpenGL coordinate system)
    droneModel = glm::rotate(droneModel, orientation.x, glm::vec3(0.0f, 0.0f, 1.0f));
    // Pitch (around X-axis in OpenGL coordinate system)  
    droneModel = glm::rotate(droneModel, orientation.y, glm::vec3(1.0f, 0.0f, 0.0f));
    // Yaw (around Y-axis in OpenGL coordinate system)
    droneModel = glm::rotate(droneModel, orientation.z, glm::vec3(0.0f, 1.0f, 0.0f));
    
    // Scale the drone
    droneModel = glm::scale(droneModel, glm::vec3(0.8f, 0.8f, 0.8f));
    
    // Set color for drone (bright blue)
    glUniform3fv(colorLoc_, 1, glm::value_ptr(color));
    glUniformMatrix4fv(modelLoc_, 1, GL_FALSE, glm::value_ptr(droneModel));
    
    // Draw drone
    glDrawArrays(GL_TRIANGLES, 0, 36);
}

void Renderer::renderSkybox() {
    // Skybox rendering (simplified for now)
    // Set sky color but don't clear here - let the main clear handle it
    // glClearColor(0.5f, 0.7f, 1.0f, 1.0f);
}

void Renderer::renderCrashMessage() {
    // Simple crash message rendering - black screen with colored rectangles
    // This will definitely work on all systems
    
    // Clear the screen to black
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // Use the existing shader program for simple text rendering
    glUseProgram(shaderProgram_);
    
    // Set up orthographic projection for 2D overlay
    glm::mat4 projection = glm::ortho(0.0f, (float)width_, (float)height_, 0.0f, -1.0f, 1.0f);
    glm::mat4 view = glm::mat4(1.0f);
    
    glUniformMatrix4fv(projectionLoc_, 1, GL_FALSE, glm::value_ptr(projection));
    glUniformMatrix4fv(viewLoc_, 1, GL_FALSE, glm::value_ptr(view));
    
    // Disable depth testing for overlay
    glDisable(GL_DEPTH_TEST);
    
    // Calculate center position
    float centerX = width_ / 2.0f;
    float centerY = height_ / 2.0f;
    
    // Draw "YOU CRASHED!" text as bright red rectangles
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(centerX, centerY - 50.0f, 0.0f));
    model = glm::scale(model, glm::vec3(300.0f, 60.0f, 1.0f));
    
    glUniformMatrix4fv(modelLoc_, 1, GL_FALSE, glm::value_ptr(model));
    glUniform3fv(colorLoc_, 1, glm::value_ptr(glm::vec3(1.0f, 0.0f, 0.0f))); // Bright red
    
    glDrawArrays(GL_TRIANGLES, 0, 36); // Draw cube
    
    // Draw "PRESS R TO RESTART" text as bright green rectangles
    model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(centerX, centerY + 50.0f, 0.0f));
    model = glm::scale(model, glm::vec3(250.0f, 40.0f, 1.0f));
    
    glUniformMatrix4fv(modelLoc_, 1, GL_FALSE, glm::value_ptr(model));
    glUniform3fv(colorLoc_, 1, glm::value_ptr(glm::vec3(0.0f, 1.0f, 0.0f))); // Bright green
    
    glDrawArrays(GL_TRIANGLES, 0, 36); // Draw cube
    
    // Re-enable depth testing
    glEnable(GL_DEPTH_TEST);
}

void Renderer::renderText(const std::string& text, float x, float y, float scale, const glm::vec3& color) {
    // Simple text rendering using basic OpenGL
    // This creates a simple bitmap font effect
    
    // Save current OpenGL state
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    
    // Disable depth testing for text overlay
    glDisable(GL_DEPTH_TEST);
    
    // Set up orthographic projection for 2D text
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, width_, height_, 0, -1, 1);
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    
    // Set text color
    glColor3f(color.r, color.g, color.b);
    
    // Simple text rendering using GL_LINES to draw characters
    glBegin(GL_LINES);
    
    float charWidth = 8.0f * scale;
    float charHeight = 12.0f * scale;
    float currentX = x;
    
    for (char c : text) {
        // Simple character rendering using lines
        // Each character is drawn as a series of line segments
        switch (c) {
            case 'Y':
                // Draw Y shape
                glVertex2f(currentX, y);
                glVertex2f(currentX + charWidth/2, y + charHeight/2);
                glVertex2f(currentX + charWidth/2, y + charHeight/2);
                glVertex2f(currentX + charWidth, y);
                glVertex2f(currentX + charWidth/2, y + charHeight/2);
                glVertex2f(currentX + charWidth/2, y + charHeight);
                break;
            case 'O':
                // Draw O shape (rectangle)
                glVertex2f(currentX, y);
                glVertex2f(currentX + charWidth, y);
                glVertex2f(currentX + charWidth, y);
                glVertex2f(currentX + charWidth, y + charHeight);
                glVertex2f(currentX + charWidth, y + charHeight);
                glVertex2f(currentX, y + charHeight);
                glVertex2f(currentX, y + charHeight);
                glVertex2f(currentX, y);
                break;
            case 'U':
                // Draw U shape
                glVertex2f(currentX, y);
                glVertex2f(currentX, y + charHeight);
                glVertex2f(currentX, y + charHeight);
                glVertex2f(currentX + charWidth, y + charHeight);
                glVertex2f(currentX + charWidth, y + charHeight);
                glVertex2f(currentX + charWidth, y);
                break;
            case 'C':
                // Draw C shape
                glVertex2f(currentX + charWidth, y);
                glVertex2f(currentX, y);
                glVertex2f(currentX, y);
                glVertex2f(currentX, y + charHeight);
                glVertex2f(currentX, y + charHeight);
                glVertex2f(currentX + charWidth, y + charHeight);
                break;
            case 'R':
                // Draw R shape
                glVertex2f(currentX, y);
                glVertex2f(currentX + charWidth, y);
                glVertex2f(currentX + charWidth, y);
                glVertex2f(currentX + charWidth, y + charHeight/2);
                glVertex2f(currentX + charWidth, y + charHeight/2);
                glVertex2f(currentX, y + charHeight/2);
                glVertex2f(currentX, y + charHeight/2);
                glVertex2f(currentX, y + charHeight);
                glVertex2f(currentX, y + charHeight);
                glVertex2f(currentX + charWidth, y + charHeight);
                break;
            case 'A':
                // Draw A shape
                glVertex2f(currentX, y + charHeight);
                glVertex2f(currentX + charWidth/2, y);
                glVertex2f(currentX + charWidth/2, y);
                glVertex2f(currentX + charWidth, y + charHeight);
                glVertex2f(currentX + charWidth/4, y + charHeight/2);
                glVertex2f(currentX + charWidth*3/4, y + charHeight/2);
                break;
            case 'S':
                // Draw S shape
                glVertex2f(currentX + charWidth, y);
                glVertex2f(currentX, y);
                glVertex2f(currentX, y);
                glVertex2f(currentX, y + charHeight/2);
                glVertex2f(currentX, y + charHeight/2);
                glVertex2f(currentX + charWidth, y + charHeight/2);
                glVertex2f(currentX + charWidth, y + charHeight/2);
                glVertex2f(currentX + charWidth, y + charHeight);
                glVertex2f(currentX + charWidth, y + charHeight);
                glVertex2f(currentX, y + charHeight);
                break;
            case 'H':
                // Draw H shape
                glVertex2f(currentX, y);
                glVertex2f(currentX, y + charHeight);
                glVertex2f(currentX, y + charHeight/2);
                glVertex2f(currentX + charWidth, y + charHeight/2);
                glVertex2f(currentX + charWidth, y + charHeight/2);
                glVertex2f(currentX + charWidth, y);
                glVertex2f(currentX + charWidth, y);
                glVertex2f(currentX + charWidth, y + charHeight);
                break;
            case 'E':
                // Draw E shape
                glVertex2f(currentX, y);
                glVertex2f(currentX + charWidth, y);
                glVertex2f(currentX, y);
                glVertex2f(currentX, y + charHeight);
                glVertex2f(currentX, y + charHeight/2);
                glVertex2f(currentX + charWidth, y + charHeight/2);
                glVertex2f(currentX, y + charHeight);
                glVertex2f(currentX + charWidth, y + charHeight);
                break;
            case 'D':
                // Draw D shape
                glVertex2f(currentX, y);
                glVertex2f(currentX, y + charHeight);
                glVertex2f(currentX, y);
                glVertex2f(currentX + charWidth*2/3, y);
                glVertex2f(currentX + charWidth*2/3, y);
                glVertex2f(currentX + charWidth, y + charHeight/2);
                glVertex2f(currentX + charWidth, y + charHeight/2);
                glVertex2f(currentX + charWidth*2/3, y + charHeight);
                glVertex2f(currentX + charWidth*2/3, y + charHeight);
                glVertex2f(currentX, y + charHeight);
                break;
            case ' ':
                // Space - just move position
                break;
            default:
                // Default character (simple rectangle)
                glVertex2f(currentX, y);
                glVertex2f(currentX + charWidth, y);
                glVertex2f(currentX + charWidth, y);
                glVertex2f(currentX + charWidth, y + charHeight);
                glVertex2f(currentX + charWidth, y + charHeight);
                glVertex2f(currentX, y + charHeight);
                glVertex2f(currentX, y + charHeight);
                glVertex2f(currentX, y);
                break;
        }
        
        currentX += charWidth + 2.0f * scale; // Add spacing between characters
    }
    
    glEnd();
    
    // Restore OpenGL state
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
    glPopAttrib();
}

unsigned int Renderer::createShader(const std::string& vertexSource, const std::string& fragmentSource) {
    unsigned int vertexShader = compileShader(GL_VERTEX_SHADER, vertexSource);
    unsigned int fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragmentSource);
    
    if (vertexShader == 0 || fragmentShader == 0) return 0;
    
    // Create shader program
    unsigned int shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    
    // Check for linking errors
    int success;
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
        std::cerr << "Shader linking failed: " << infoLog << std::endl;
        return 0;
    }
    
    // Clean up
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    
    return shaderProgram;
}

unsigned int Renderer::compileShader(unsigned int type, const std::string& source) {
    unsigned int shader = glCreateShader(type);
    const char* src = source.c_str();
    glShaderSource(shader, 1, &src, NULL);
    glCompileShader(shader);
    
    // Check for compilation errors
    int success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetShaderInfoLog(shader, 512, NULL, infoLog);
        std::cerr << "Shader compilation failed: " << infoLog << std::endl;
        return 0;
    }
    
    return shader;
}
