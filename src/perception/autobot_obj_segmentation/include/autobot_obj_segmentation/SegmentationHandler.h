#ifndef SEGMENTATION_HANDLER_H
#define SEGMENTATION_HANDLER_H

#include <mutex>
#include <memory>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "autobot_msgs/msg/detections2_d.hpp"
#include "autobot_msgs/msg/object_observations.hpp"

class SegmentationHandler
{
public:
    SegmentationHandler();
    ~SegmentationHandler();

private:
    void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr aInfo);
    void onSynced(const sensor_msgs::msg::Image::ConstSharedPtr& aDepth,
                  const autobot_msgs::msg::Detections2D::ConstSharedPtr& aDetections);

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, autobot_msgs::msg::Detections2D>;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

    message_filters::Subscriber<sensor_msgs::msg::Image> mDepthSub;
    message_filters::Subscriber<autobot_msgs::msg::Detections2D> mDetectionsSub;
    std::unique_ptr<Synchronizer> mSync;

    std::mutex mCameraInfoMutex;
    sensor_msgs::msg::CameraInfo::SharedPtr mLatestCameraInfo;  // nullptr until first msg
};
#endif // SEGMENTATION_HANDLER_H
