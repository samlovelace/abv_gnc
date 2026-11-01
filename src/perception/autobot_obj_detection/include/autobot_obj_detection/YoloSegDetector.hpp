#pragma once
#include "autobot_obj_detection/OrtDetector.hpp"

// Ultralytics YOLO11-seg, ONNX export (yolo export ... format=onnx imgsz=640
// simplify=True opset=12 - static shapes). See YoloSegDetector.cpp for the
// exact expected model I/O shapes.
class YoloSegDetector : public OrtDetector {
public:
    explicit YoloSegDetector(const DetectorConfig& cfg) : OrtDetector(cfg) {}

protected:
    std::vector<Tensor> preprocess(const cv::Mat& bgr, FrameCtx& ctx) override;
    Detections postprocess(const std::vector<Tensor>& outs, const FrameCtx& ctx) override;
};
