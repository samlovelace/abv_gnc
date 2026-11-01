#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <plog/Log.h>

#include "abv_common/ConfigurationManager.h"
#include "abv_common/RosTopicManager.h"
#include "autobot_obj_detection/DetectorFactory.hpp"
#include "autobot_obj_detection/ObjectDetectionHandler.h"
#include "abv_msgs/msg/detections2_d.hpp"

namespace {

sensor_msgs::msg::RegionOfInterest boxToRoiMsg(const cv::Rect& box)
{
    sensor_msgs::msg::RegionOfInterest roi;
    roi.x_offset = static_cast<uint32_t>(box.x);
    roi.y_offset = static_cast<uint32_t>(box.y);
    roi.width = static_cast<uint32_t>(box.width);
    roi.height = static_cast<uint32_t>(box.height);
    roi.do_rectify = false;
    return roi;
}

sensor_msgs::msg::Image maskToImageMsg(const cv::Mat& mask, const std_msgs::msg::Header& header)
{
    // empty (zero-size) Image for box-only models, matching an empty cv::Mat
    if (mask.empty()) return sensor_msgs::msg::Image();
    return *cv_bridge::CvImage(header, "mono8", mask).toImageMsg();
}

}  // namespace

ObjectDetectionHandler::ObjectDetectionHandler()
{
    const auto& cfg = ConfigurationManager::getInstance()->getObjDetectionConfig();

    DetectorConfig detectorCfg;
    detectorCfg.type = cfg.mType;
    detectorCfg.modelPath = cfg.mModelPath;
    detectorCfg.classNames = cfg.mClassNames;
    detectorCfg.inputSize = cfg.mInputSize;
    detectorCfg.confThresh = static_cast<float>(cfg.mConfThresh);
    detectorCfg.iouThresh = static_cast<float>(cfg.mIouThresh);
    detectorCfg.useCuda = cfg.mUseCuda;

    mDetector = makeDetector(detectorCfg);

    RosTopicManager::getInstance()->createSubscriber<sensor_msgs::msg::Image>("/camera/image_raw",
        std::bind(&ObjectDetectionHandler::frameCallback, this, std::placeholders::_1));

    RosTopicManager::getInstance()->createPublisher<abv_msgs::msg::Detections2D>("/detections");

    mRunning = true;
    mWorkerThread = std::thread(&ObjectDetectionHandler::run, this);
}

ObjectDetectionHandler::~ObjectDetectionHandler()
{
    mRunning = false;
    if (mWorkerThread.joinable()) mWorkerThread.join();
}

void ObjectDetectionHandler::run()
{
    LOGI << "Starting object detection run loop...";

    while (mRunning && rclcpp::ok())
    {
        cv::Mat frame;
        if (!mFrameQueue.waitPop(frame, std::chrono::milliseconds(100))) continue;

        Detections dets = mDetector->detect(frame);

        std_msgs::msg::Header header;
        header.stamp = RosTopicManager::getInstance()->now();

        abv_msgs::msg::Detections2D msg;
        msg.header = header;
        msg.detections.reserve(dets.size());

        for (const auto& d : dets)
        {
            abv_msgs::msg::Detection2D out;
            out.label = d.label;
            out.score = d.score;
            out.box = boxToRoiMsg(d.box);
            out.mask = maskToImageMsg(d.mask, header);
            msg.detections.push_back(std::move(out));
        }

        RosTopicManager::getInstance()->publishMessage("/detections", msg);
    }
}

void ObjectDetectionHandler::frameCallback(const sensor_msgs::msg::Image::SharedPtr aFrame)
{
    cv::Mat view;

    if (aFrame->encoding == "bgr8")
    {
        view = cv::Mat(aFrame->height, aFrame->width, CV_8UC3,
                       const_cast<uint8_t*>(aFrame->data.data()), aFrame->step);
    }
    else if (aFrame->encoding == "rgb8")
    {
        cv::Mat rgb(aFrame->height, aFrame->width, CV_8UC3,
                    const_cast<uint8_t*>(aFrame->data.data()), aFrame->step);
        cv::cvtColor(rgb, view, cv::COLOR_RGB2BGR);
    }
    else
    {
        throw std::runtime_error("unsupported encoding: " + aFrame->encoding);
    }

    // clone so we own the data - view may just be wrapping aFrame's buffer
    mFrameQueue.push(view.clone());
}
