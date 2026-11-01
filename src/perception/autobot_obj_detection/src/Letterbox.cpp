#include "autobot_obj_detection/Letterbox.hpp"
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <stdexcept>

Tensor letterboxToTensor(const cv::Mat& bgr, int S, FrameCtx& ctx)
{
    if (bgr.empty())
        throw std::invalid_argument("letterboxToTensor: input image is empty");
    if (bgr.type() != CV_8UC3)
        throw std::invalid_argument("letterboxToTensor: input image must be CV_8UC3");

    ctx.scale = std::min(S / (float)bgr.cols, S / (float)bgr.rows);
    int w = std::round(bgr.cols * ctx.scale), h = std::round(bgr.rows * ctx.scale);
    ctx.padX = (S - w) / 2;
    ctx.padY = (S - h) / 2;

    cv::Mat resized, canvas(S, S, CV_8UC3, cv::Scalar(114, 114, 114));
    cv::resize(bgr, resized, {w, h});
    resized.copyTo(canvas(cv::Rect(ctx.padX, ctx.padY, w, h)));

    cv::Mat f;
    canvas.convertTo(f, CV_32FC3, 1.0 / 255.0);

    Tensor t{{1, 3, S, S}, std::vector<float>(3 * S * S)};
    std::vector<cv::Mat> planes = {
        cv::Mat(S, S, CV_32F, t.data.data() + 2 * S * S),   // B -> channel 2
        cv::Mat(S, S, CV_32F, t.data.data() + 1 * S * S),   // G -> channel 1
        cv::Mat(S, S, CV_32F, t.data.data() + 0 * S * S)};  // R -> channel 0
    cv::split(f, planes);
    return t;
}
