#pragma once
#include "autobot_obj_detection/Detection.hpp"

class IDetector {
public:
    virtual ~IDetector() = default;
    // Input: BGR cv::Mat (OpenCV convention), any size. Output: detections in input-image coordinates.
    virtual Detections detect(const cv::Mat& bgr) = 0;
};
