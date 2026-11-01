#include "autobot_obj_detection/YoloSegDetector.hpp"
#include "autobot_obj_detection/Letterbox.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {

float iou(const cv::Rect& a, const cv::Rect& b)
{
    int interArea = (a & b).area();
    int unionArea = a.area() + b.area() - interArea;
    return unionArea > 0 ? (float)interArea / unionArea : 0.f;
}

// mask_lowres = sigmoid(coeffs[1x32] * protos[32 x (mh*mw)]), resized up to
// the letterbox canvas, cropped back to the unpadded region, resized to the
// original image, then thresholded and masked down to just the box.
cv::Mat buildMask(const Tensor& protos, const float* coeffs32, const FrameCtx& ctx,
                   const cv::Rect& box, int canvasSize)
{
    int mh = static_cast<int>(protos.shape[2]);
    int mw = static_cast<int>(protos.shape[3]);

    cv::Mat protosMat(32, mh * mw, CV_32F, const_cast<float*>(protos.data.data()));
    cv::Mat coeffsMat(1, 32, CV_32F, const_cast<float*>(coeffs32));
    cv::Mat lowRes = cv::Mat(coeffsMat * protosMat).reshape(1, mh);

    cv::Mat neg, sig;
    cv::exp(-lowRes, neg);
    sig = 1.0f / (1.0f + neg);

    cv::Mat upFull;
    cv::resize(sig, upFull, {canvasSize, canvasSize});

    int cropW = canvasSize - 2 * ctx.padX;
    int cropH = canvasSize - 2 * ctx.padY;
    cv::Mat cropped = upFull(cv::Rect(ctx.padX, ctx.padY, cropW, cropH));

    cv::Mat full;
    cv::resize(cropped, full, ctx.origSize);

    cv::Mat thresholded = full > 0.5f;   // CV_8U, 0/255

    cv::Mat result = cv::Mat::zeros(ctx.origSize, CV_8UC1);
    thresholded(box).copyTo(result(box));
    return result;
}

}  // namespace

std::vector<Tensor> YoloSegDetector::preprocess(const cv::Mat& bgr, FrameCtx& ctx)
{
    return { letterboxToTensor(bgr, mCfg.inputSize, ctx) };
}

Detections YoloSegDetector::postprocess(const std::vector<Tensor>& outs, const FrameCtx& ctx)
{
    if (outs.size() != 2)
        throw std::runtime_error("YoloSegDetector: expected 2 outputs, got " +
                                  std::to_string(outs.size()));

    const Tensor& det    = outs[0];
    const Tensor& protos = outs[1];

    if (det.shape.size() != 3 || protos.shape.size() != 4)
        throw std::runtime_error("YoloSegDetector: unexpected output rank");

    const int C  = static_cast<int>(det.shape[1]);
    const int N  = static_cast<int>(det.shape[2]);
    const int nc = static_cast<int>(mCfg.classNames.size());
    const int expectedC = 4 + nc + 32;

    if (C != expectedC)
        throw std::runtime_error(
            "YoloSegDetector: expected " + std::to_string(expectedC) + " channels (4 + " +
            std::to_string(nc) + " classes + 32 mask coeffs), got " + std::to_string(C));

    struct Candidate { int anchor; int cls; float score; cv::Rect box; };
    std::vector<Candidate> candidates;

    const float* data = det.data.data();
    const int canvasSize = mCfg.inputSize;

    for (int i = 0; i < N; ++i) {
        int bestCls = -1;
        float bestScore = 0.f;
        for (int c = 0; c < nc; ++c) {
            float s = data[(4 + c) * N + i];
            if (s > bestScore) { bestScore = s; bestCls = c; }
        }
        if (bestCls < 0 || bestScore < mCfg.confThresh) continue;

        float cx = data[0 * N + i], cy = data[1 * N + i];
        float bw = data[2 * N + i], bh = data[3 * N + i];

        float x1 = (cx - bw / 2.f - ctx.padX) / ctx.scale;
        float y1 = (cy - bh / 2.f - ctx.padY) / ctx.scale;
        float x2 = (cx + bw / 2.f - ctx.padX) / ctx.scale;
        float y2 = (cy + bh / 2.f - ctx.padY) / ctx.scale;

        x1 = std::clamp(x1, 0.f, (float)ctx.origSize.width - 1);
        y1 = std::clamp(y1, 0.f, (float)ctx.origSize.height - 1);
        x2 = std::clamp(x2, 0.f, (float)ctx.origSize.width - 1);
        y2 = std::clamp(y2, 0.f, (float)ctx.origSize.height - 1);

        cv::Rect box(cv::Point((int)x1, (int)y1), cv::Point((int)x2, (int)y2));
        candidates.push_back({i, bestCls, bestScore, box});
    }

    // class-aware greedy NMS (avoids linking opencv_dnn just for NMSBoxes)
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b) { return a.score > b.score; });

    std::vector<bool> suppressed(candidates.size(), false);
    std::vector<int> keep;
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (suppressed[i]) continue;
        keep.push_back(static_cast<int>(i));
        for (size_t j = i + 1; j < candidates.size(); ++j) {
            if (suppressed[j] || candidates[j].cls != candidates[i].cls) continue;
            if (iou(candidates[i].box, candidates[j].box) > mCfg.iouThresh) suppressed[j] = true;
        }
    }

    Detections result;
    result.reserve(keep.size());
    for (int idx : keep) {
        const Candidate& cand = candidates[idx];

        std::vector<float> coeffs(32);
        for (int k = 0; k < 32; ++k) coeffs[k] = data[(4 + nc + k) * N + cand.anchor];

        Detection d;
        d.label = mCfg.classNames[cand.cls];
        d.score = cand.score;
        d.box   = cand.box;
        d.mask  = buildMask(protos, coeffs.data(), ctx, cand.box, canvasSize);
        result.push_back(std::move(d));
    }
    return result;
}
