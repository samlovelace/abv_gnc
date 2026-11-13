#ifndef OBJECT_DETECTION_HANDLER
#define OBJECT_DETECTION_HANDLER

#include <atomic>
#include <memory>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <thread>

#include "autobot_common/BoundedQueue.hpp"
#include "autobot_obj_detection/IDetector.hpp"

/**
 * @brief Responsible for subscribing to camera RGB images, running object detection, publishing those results
 */
class ObjectDetectionHandler
{
public:
    ObjectDetectionHandler();
    ~ObjectDetectionHandler();

private:
    struct StampedFrame
    {
        cv::Mat frame;
        std_msgs::msg::Header header;
    };

    void run();
    void frameCallback(const sensor_msgs::msg::Image::SharedPtr aFrame);

    BoundedQueue<StampedFrame> mFrameQueue;
    std::unique_ptr<IDetector> mDetector;

    std::thread mWorkerThread;
    std::atomic<bool> mRunning{false};
};
#endif //OBJECT_DETECTION_HANDLER
