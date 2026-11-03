#pragma once
#include "autobot_common/OrtModel.hpp"
#include "autobot_obj_detection/IDetector.hpp"

class OrtDetector : public IDetector {
public:
    Detections detect(const cv::Mat& bgr) final        // pre -> infer -> post, written once
    {
        FrameCtx ctx;
        ctx.origSize = bgr.size();
        auto inputs  = preprocess(bgr, ctx);
        auto outputs = mModel.run(inputs);
        return postprocess(outputs, ctx);
    }

protected:
    explicit OrtDetector(const DetectorConfig& cfg)
        : mCfg(cfg), mModel(cfg.modelPath, {cfg.useCuda}) {}

    virtual std::vector<Tensor> preprocess(const cv::Mat& bgr, FrameCtx& ctx) = 0;
    virtual Detections postprocess(const std::vector<Tensor>& outs, const FrameCtx& ctx) = 0;

    DetectorConfig mCfg;   // declared before mModel: matches initializer order
    OrtModel       mModel;
};
