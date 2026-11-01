#pragma once
#include <opencv2/core.hpp>
#include "abv_common/Tensor.hpp"
#include "autobot_obj_detection/Detection.hpp"

// Letterbox-resizes bgr into an S x S canvas (padded with 114/255 gray),
// returns a {1,3,S,S} CHW, RGB-ordered float tensor normalized to [0,1].
// Throws std::invalid_argument on an empty input or a non-CV_8UC3 mat.
Tensor letterboxToTensor(const cv::Mat& bgr, int size, FrameCtx& ctx);
