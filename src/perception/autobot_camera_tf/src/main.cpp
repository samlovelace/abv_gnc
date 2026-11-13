#include <cstdio>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "autobot_common/RosTopicManager.h"
#include "autobot_common/DataLogger.h"
#include "autobot_common/SignalHandler.hpp"
#include "autobot_common/ConfigurationManager.h"
#include "autobot_common/HeartbeatPublisher.h"

int main()
{
    std::signal(SIGINT, signalHandler);

    const std::string name = "camera_tf";

    DataLogger::get().createMainLog(name);
    ConfigurationManager::getInstance()->loadConfiguration();

    rclcpp::init(0, nullptr);
    RosTopicManager::getInstance(name);
    RosTopicManager::getInstance()->spinNode();
    HeartbeatPublisher heartbeat(name);

    const auto& cfg = ConfigurationManager::getInstance()->getCameraExtrinsicsConfig();

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = RosTopicManager::getInstance()->now();
    transform.header.frame_id = cfg.mWorldFrame;
    transform.child_frame_id = cfg.mCameraFrame;
    transform.transform.translation.x = cfg.mTranslationXYZ.x();
    transform.transform.translation.y = cfg.mTranslationXYZ.y();
    transform.transform.translation.z = cfg.mTranslationXYZ.z();
    transform.transform.rotation.x = cfg.mRotationQuaternionXYZW[0];
    transform.transform.rotation.y = cfg.mRotationQuaternionXYZW[1];
    transform.transform.rotation.z = cfg.mRotationQuaternionXYZW[2];
    transform.transform.rotation.w = cfg.mRotationQuaternionXYZW[3];

    tf2_ros::StaticTransformBroadcaster broadcaster(RosTopicManager::getInstance());
    broadcaster.sendTransform(transform);

    while(rclcpp::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    rclcpp::shutdown();
}
