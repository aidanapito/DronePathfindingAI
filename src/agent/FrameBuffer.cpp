#include "agent/VisionAgent.h"

namespace agent {

void FrameBuffer::addFrame(const cv::Mat& frame) {
    frames.push_back(frame.clone());
    
    // Keep only the most recent frames
    while (frames.size() > max_frames) {
        frames.pop_front();
    }
}

cv::Mat FrameBuffer::getStackedFrames() const {
    if (frames.empty()) {
        return cv::Mat();
    }
    
    // Create a stacked representation
    // For now, just return the most recent frame
    // TODO: Implement proper frame stacking
    return frames.back();
}

void FrameBuffer::reset() {
    frames.clear();
}

} // namespace agent
