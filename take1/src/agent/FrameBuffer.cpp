#include "agent/VisionAgent.h"
#include "agent/Agent.h"
#include <opencv2/opencv.hpp>

namespace agent {

void FrameBuffer::addFrame(const cv::Mat& frame) {
    // Ensure frame is in the correct format (grayscale, 84x84)
    cv::Mat processed_frame;
    
    // Convert to grayscale if it's a 3-channel image
    if (frame.channels() == 3) {
        cv::cvtColor(frame, processed_frame, cv::COLOR_BGR2GRAY);
    } else if (frame.channels() == 1) {
        processed_frame = frame.clone();
    } else {
        // Handle other channel counts by converting to grayscale
        cv::cvtColor(frame, processed_frame, cv::COLOR_BGR2GRAY);
    }
    
    // Resize to standard input size (84x84 for most vision models)
    cv::resize(processed_frame, processed_frame, cv::Size(84, 84));
    
    frames.push_back(processed_frame);
    
    // Keep only the most recent frames
    while (frames.size() > max_frames) {
        frames.pop_front();
    }
}

cv::Mat FrameBuffer::getStackedFrames() const {
    if (frames.empty()) {
        return cv::Mat();
    }
    
    if (frames.size() == 1) {
        // Single frame - return as is
        return frames.back();
    }
    
    // Create a stacked representation by concatenating frames along channels
    // This creates a multi-channel image where each channel represents a time step
    std::vector<cv::Mat> channels;
    
    // Pad with zeros if we don't have enough frames
    int frames_needed = max_frames;
    for (int i = 0; i < frames_needed; ++i) {
        if (i < frames.size()) {
            channels.push_back(frames[i]);
        } else {
            // Add zero frame for missing time steps
            cv::Mat zero_frame = cv::Mat::zeros(84, 84, CV_8UC1);
            channels.push_back(zero_frame);
        }
    }
    
    // Merge channels into a single multi-channel image
    cv::Mat stacked_frames;
    cv::merge(channels, stacked_frames);
    
    return stacked_frames;
}

void FrameBuffer::reset() {
    frames.clear();
}

} // namespace agent
