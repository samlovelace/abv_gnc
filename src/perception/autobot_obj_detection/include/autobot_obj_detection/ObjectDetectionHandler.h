#ifndef OBJECT_DETECTION_HANDLER
#define OBJECT_DETECTION_HANDLER

#include <atomic>
#include <memory>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>

#include "abv_common/BoundedQueue.hpp"
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
    void run();
    void frameCallback(const sensor_msgs::msg::Image::SharedPtr aFrame);

    BoundedQueue<cv::Mat> mFrameQueue;
    std::unique_ptr<IDetector> mDetector;

    std::thread mWorkerThread;
    std::atomic<bool> mRunning{false};
};
#endif //OBJECT_DETECTION_HANDLER
