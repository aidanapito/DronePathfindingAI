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
        glm::vec3 position(obstacle.x, obstacle.y, obstacle.z);
        
        switch (obstacle.type) {
            case ObstacleType::SKYSCRAPER:
                color = glm::vec3(0.6f, 0.6f, 0.6f); // Gray
                size = glm::vec3(40.0f, 40.0f, 100.0f);
                break;
            case ObstacleType::GROUND_OBSTACLE:
                color = glm::vec3(0.55f, 0.27f, 0.07f); // Brown
                size = glm::vec3(30.0f, 30.0f, 20.0f);
                break;
            case ObstacleType::MOUNTAIN:
                color = glm::vec3(0.4f, 0.4f, 0.4f); // Dark gray
                size = glm::vec3(80.0f, 80.0f, 60.0f);
                break;
            default:
                color = glm::vec3(1.0f, 1.0f, 1.0f); // White
                size = glm::vec3(20.0f, 20.0f, 20.0f);
                break;
        }
        
        renderCube(position, size, color);
    }
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
    
    // Ground vertices
    float groundVertices[] = {
        -1000.0f, 0.0f, -1000.0f,
         1000.0f, 0.0f, -1000.0f,
         1000.0f, 0.0f,  1000.0f,
        -1000.0f, 0.0f,  1000.0f
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

void Renderer::renderGround() {
    glBindVertexArray(groundVAO_);
    
    // Set ground color
    glUniform3fv(colorLoc_, 1, glm::value_ptr(glm::vec3(0.2f, 0.8f, 0.2f)));
    
    // Create ground model matrix
    glm::mat4 model = glm::mat4(1.0f);
    glUniformMatrix4fv(modelLoc_, 1, GL_FALSE, glm::value_ptr(model));
    
    // Draw ground
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
}

void Renderer::renderSkybox() {
    // Skybox rendering (simplified for now)
    glClearColor(0.5f, 0.7f, 1.0f, 1.0f);
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
