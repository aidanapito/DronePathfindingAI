#include "agent/VisionAgent.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <random>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace agent {

VisionAgent::VisionAgent(const AgentConfig& config) 
    : Agent(config), last_inference_time_(0.0f) {
    frame_buffer_.max_frames = config.observation_stack;
    
    // Initialize action preferences
    action_preferences_ = {1.0f, 1.0f, 1.0f, 1.0f}; // FORWARD, LEFT, RIGHT, IDLE
    
    // Initialize with random exploration
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(0.0f, 1.0f);
    
    // For now, we'll use a simple heuristic-based approach
    // In a full implementation, this would load a pre-trained model
    std::cout << "VisionAgent initialized with " << config.observation_stack << " frame stack" << std::endl;
}

Action VisionAgent::selectAction(const Observation& obs, const sim::Drone& drone) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Update frame buffer with current observation
    if (!obs.image.empty()) {
        updateFrameBuffer(obs.image);
    }
    
    Action selected_action;
    
    // Get stacked frames for temporal information
    cv::Mat stacked_frames = frame_buffer_.getStackedFrames();
    
    if (!stacked_frames.empty()) {
        // Use vision-based inference if we have frames
        selected_action = inferAction(stacked_frames);
    } else {
        // Fallback to heuristic-based action selection
        selected_action = selectHeuristicAction(obs, drone);
    }
    
    // Measure inference time
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    last_inference_time_ = duration.count() / 1000.0f; // Convert to milliseconds
    
    return selected_action;
}

void VisionAgent::updatePolicy(const Observation& obs, Action action, 
                              float reward, const Observation& next_obs, bool done) {
    // For now, implement a simple experience replay buffer
    // In a full implementation, this would update the neural network weights
    
    // Store experience for potential future training
    Experience exp;
    exp.observation = obs;
    exp.action = action;
    exp.reward = reward;
    exp.next_observation = next_obs;
    exp.done = done;
    
    experience_buffer_.push_back(exp);
    
    // Keep buffer size manageable
    if (experience_buffer_.size() > 10000) {
        experience_buffer_.erase(experience_buffer_.begin());
    }
    
    // Simple learning: adjust action preferences based on rewards
    if (reward > 0) {
        // Positive reward - reinforce this action
        action_preferences_[static_cast<int>(action)] += reward * 0.1f;
    } else if (reward < 0) {
        // Negative reward - discourage this action
        action_preferences_[static_cast<int>(action)] += reward * 0.05f;
    }
    
    // Normalize preferences to prevent explosion
    float max_pref = *std::max_element(action_preferences_.begin(), action_preferences_.end());
    if (max_pref > 10.0f) {
        for (auto& pref : action_preferences_) {
            pref /= max_pref / 10.0f;
        }
    }
}

void VisionAgent::reset() {
    frame_buffer_.reset();
    // Don't clear experience buffer - keep learned knowledge
}

Action VisionAgent::inferAction(const cv::Mat& stacked_frames) {
    // Enhanced vision-based action selection with goal guidance
    cv::Mat processed = preprocessFrame(stacked_frames);
    
    // Convert back to 8-bit for edge detection
    cv::Mat processed_8u;
    processed.convertTo(processed_8u, CV_8UC1, 255.0);
    
    // Simple edge detection to find obstacles
    cv::Mat edges;
    cv::Canny(processed_8u, edges, 50, 150);
    
    // Count edge pixels in different regions
    int left_edges = cv::countNonZero(edges(cv::Rect(0, 0, 28, 84)));
    int center_edges = cv::countNonZero(edges(cv::Rect(28, 0, 28, 84)));
    int right_edges = cv::countNonZero(edges(cv::Rect(56, 0, 28, 84)));
    
    // Enhanced obstacle avoidance logic with goal guidance
    if (center_edges > 100) {
        // Obstacle ahead - turn toward clearer side
        if (left_edges < right_edges) {
            return Action::YAW_LEFT;
        } else {
            return Action::YAW_RIGHT;
        }
    } else if (left_edges > 150) {
        // Obstacle on left - turn right
        return Action::YAW_RIGHT;
    } else if (right_edges > 150) {
        // Obstacle on right - turn left
        return Action::YAW_LEFT;
    } else {
        // Clear path ahead - move forward
        return Action::THROTTLE_FORWARD;
    }
}

Action VisionAgent::selectHeuristicAction(const Observation& obs, const sim::Drone& drone) {
    // Enhanced heuristic-based action selection with better goal guidance
    
    // Use goal direction to guide movement
    float goal_direction = obs.goal_direction;
    float current_heading = obs.heading;
    
    // Calculate heading difference
    float heading_diff = goal_direction - current_heading;
    
    // Normalize to [-π, π]
    while (heading_diff > M_PI) heading_diff -= 2 * M_PI;
    while (heading_diff < -M_PI) heading_diff += 2 * M_PI;
    
    // Enhanced action selection based on heading alignment and distance
    float distance_to_goal = obs.distance_to_goal;
    
    if (distance_to_goal < 50.0f) {
        // Very close to goal - be more precise
        if (std::abs(heading_diff) < 0.2f) {
            return Action::THROTTLE_FORWARD;
        } else if (heading_diff > 0) {
            return Action::YAW_LEFT;
        } else {
            return Action::YAW_RIGHT;
        }
    } else if (distance_to_goal < 150.0f) {
        // Moderately close to goal - balance precision and speed
        if (std::abs(heading_diff) < 0.3f) {
            return Action::THROTTLE_FORWARD;
        } else if (heading_diff > 0) {
            return Action::YAW_LEFT;
        } else {
            return Action::YAW_RIGHT;
        }
    } else {
        // Far from goal - be more aggressive with turning
        if (std::abs(heading_diff) < 0.4f) {
            return Action::THROTTLE_FORWARD;
        } else if (heading_diff > 0.1f) {
            return Action::YAW_LEFT;
        } else if (heading_diff < -0.1f) {
            return Action::YAW_RIGHT;
        } else {
            return Action::THROTTLE_FORWARD;
        }
    }
}

cv::Mat VisionAgent::preprocessFrame(const cv::Mat& frame) const {
    cv::Mat processed = frame.clone();
    
    // Ensure we have a single-channel 8-bit image
    if (processed.channels() != 1) {
        cv::cvtColor(processed, processed, cv::COLOR_BGR2GRAY);
    }
    
    // Ensure it's 8-bit
    if (processed.depth() != CV_8U) {
        processed.convertTo(processed, CV_8UC1);
    }
    
    // Apply simple preprocessing
    // 1. Gaussian blur to reduce noise
    cv::GaussianBlur(processed, processed, cv::Size(3, 3), 0);
    
    // 2. Histogram equalization for better contrast
    cv::equalizeHist(processed, processed);
    
    // 3. Convert to float and normalize to [0, 1]
    processed.convertTo(processed, CV_32F, 1.0/255.0);
    
    // 4. Normalize to zero mean and unit variance
    cv::Scalar mean, stddev;
    cv::meanStdDev(processed, mean, stddev);
    processed = (processed - mean[0]) / (stddev[0] + 1e-8);
    
    return processed;
}

cv::Mat VisionAgent::addGoalDirection(const cv::Mat& frame, float goal_direction) const {
    cv::Mat annotated = frame.clone();
    
    if (annotated.channels() == 1) {
        cv::cvtColor(annotated, annotated, cv::COLOR_GRAY2BGR);
    }
    
    // Calculate arrow endpoints
    int center_x = annotated.cols / 2;
    int center_y = annotated.rows / 2;
    int arrow_length = 30;
    
    int end_x = center_x + static_cast<int>(arrow_length * cos(goal_direction));
    int end_y = center_y + static_cast<int>(arrow_length * sin(goal_direction));
    
    // Draw goal direction arrow
    cv::arrowedLine(annotated, cv::Point(center_x, center_y), 
                    cv::Point(end_x, end_y), cv::Scalar(0, 255, 0), 2);
    
    return annotated;
}

void VisionAgent::updateFrameBuffer(const cv::Mat& frame) {
    frame_buffer_.addFrame(frame);
}

cv::Mat VisionAgent::normalizeFrame(const cv::Mat& frame) const {
    cv::Mat normalized = frame.clone();
    
    // Convert to float
    normalized.convertTo(normalized, CV_32F);
    
    // Normalize to [0, 1] range
    cv::normalize(normalized, normalized, 0.0, 1.0, cv::NORM_MINMAX);
    
    return normalized;
}

std::vector<float> VisionAgent::frameToTensor(const cv::Mat& frame) const {
    std::vector<float> tensor;
    
    // Reshape frame to 1D vector
    cv::Mat flattened = frame.reshape(1, 1);
    flattened.convertTo(flattened, CV_32F);
    
    // Copy to vector
    tensor.assign((float*)flattened.data, (float*)flattened.data + flattened.total());
    
    return tensor;
}

Action VisionAgent::tensorToAction(const std::vector<float>& action_probs) const {
    if (action_probs.empty()) {
        return Action::IDLE;
    }
    
    // Find the action with highest probability
    auto max_it = std::max_element(action_probs.begin(), action_probs.end());
    int action_index = std::distance(action_probs.begin(), max_it);
    
    // Map index to action
    switch (action_index) {
        case 0: return Action::THROTTLE_FORWARD;
        case 1: return Action::YAW_LEFT;
        case 2: return Action::YAW_RIGHT;
        case 3: return Action::IDLE;
        default: return Action::IDLE;
    }
}

void VisionAgent::saveModel(const std::string& path) {
    // For now, save action preferences and experience buffer
    // In a full implementation, this would save the neural network weights
    
    std::cout << "VisionAgent: Saving model to " << path << std::endl;
    std::cout << "Action preferences: ";
    for (size_t i = 0; i < action_preferences_.size(); ++i) {
        std::cout << action_preferences_[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "Experience buffer size: " << experience_buffer_.size() << std::endl;
}

void VisionAgent::loadModel(const std::string& path) {
    // For now, load default preferences
    // In a full implementation, this would load the neural network weights
    
    std::cout << "VisionAgent: Loading model from " << path << std::endl;
    
    // Initialize with default action preferences
    action_preferences_ = {1.0f, 1.0f, 1.0f, 1.0f}; // FORWARD, LEFT, RIGHT, IDLE
}

bool VisionAgent::loadONNXModel(const std::string& path) {
    // TODO: Implement ONNX model loading
    std::cout << "ONNX model loading not yet implemented" << std::endl;
    return false;
}

bool VisionAgent::loadTorchModel(const std::string& path) {
    // TODO: Implement Torch model loading
    std::cout << "Torch model loading not yet implemented" << std::endl;
    return false;
}

std::vector<float> VisionAgent::runONNXInference(const cv::Mat& input) {
    // TODO: Implement ONNX inference
    return std::vector<float>();
}

std::vector<float> VisionAgent::runTorchInference(const cv::Mat& input) {
    // TODO: Implement Torch inference
    return std::vector<float>();
}

} // namespace agent
