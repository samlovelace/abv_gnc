#pragma once
#include <opencv2/core.hpp>
#include <string>
#include <vector>

struct Detection {
    std::string label;
    float       score = 0.f;
    cv::Rect    box;          // in ORIGINAL image pixel coordinates, clamped to the image
    cv::Mat     mask;         // CV_8UC1, original image size, 0 or 255; empty for box-only models
};
using Detections = std::vector<Detection>;

// What postprocess needs to map model output back onto the original image.
struct FrameCtx {
    cv::Size origSize;
    float    scale = 1.f;     // letterbox scale
    int      padX = 0, padY = 0;
};

struct DetectorConfig {
    std::string type;                         // "yolo11-seg", later "rfdetr", "yoloe", ...
    std::string modelPath;
    std::vector<std::string> classNames;      // index == class channel in the model output
    int   inputSize  = 640;
    float confThresh = 0.25f;
    float iouThresh  = 0.45f;
    bool  useCuda    = false;
};
