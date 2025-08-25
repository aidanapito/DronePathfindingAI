#pragma once

#include "Agent.h"
#include <vector>
#include <deque>

#ifdef USE_ONNX
#include <onnxruntime_cxx_api.h>
#elif defined(USE_LIBTORCH)
#include <torch/torch.h>
#endif

namespace agent {

struct FrameBuffer {
    std::deque<cv::Mat> frames;
    int max_frames;
    
    void addFrame(const cv::Mat& frame);
    cv::Mat getStackedFrames() const;
    void reset();
};

class VisionAgent : public Agent {
public:
    VisionAgent(const AgentConfig& config);
    ~VisionAgent() override = default;

    // Agent interface implementation
    Action selectAction(const Observation& obs, const sim::Drone& drone) override;
    void updatePolicy(const Observation& obs, Action action, 
                     float reward, const Observation& next_obs, bool done) override;
    void reset() override;
    
    // Model inference
    Action inferAction(const cv::Mat& stacked_frames);
    
    // Model persistence
    void saveModel(const std::string& path) override;
    void loadModel(const std::string& path) override;
    
    // Vision-specific methods
    cv::Mat preprocessFrame(const cv::Mat& frame) const;
    cv::Mat addGoalDirection(const cv::Mat& frame, float goal_direction) const;
    void updateFrameBuffer(const cv::Mat& frame);
    
    // Statistics
    float getInferenceTime() const { return last_inference_time_; }
    int getFrameBufferSize() const { return frame_buffer_.frames.size(); }

private:
    FrameBuffer frame_buffer_;
    
    // Model inference
    #ifdef USE_ONNX
    std::unique_ptr<Ort::Session> onnx_session_;
    std::vector<const char*> input_names_;
    std::vector<const char*> output_names_;
    #elif defined(USE_LIBTORCH)
    torch::jit::script::Module torch_model_;
    #endif
    
    // Performance tracking
    float last_inference_time_;
    
    // Helper methods
    cv::Mat normalizeFrame(const cv::Mat& frame) const;
    std::vector<float> frameToTensor(const cv::Mat& frame) const;
    Action tensorToAction(const std::vector<float>& action_probs) const;
    
    // Model loading
    bool loadONNXModel(const std::string& path);
    bool loadTorchModel(const std::string& path);
    
    // Inference engines
    std::vector<float> runONNXInference(const cv::Mat& input);
    std::vector<float> runTorchInference(const cv::Mat& input);
};

} // namespace agent
