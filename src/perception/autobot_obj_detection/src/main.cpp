
#include <cstdio>
#include "abv_common/RosTopicManager.h"
#include "abv_common/DataLogger.h"
#include "abv_common/SignalHandler.hpp"
#include "abv_common/ConfigurationManager.h"
#include "abv_common/HeartbeatPublisher.h"
#include "autobot_obj_detection/ObjectDetectionHandler.h"

int main()
{
    std::signal(SIGINT, signalHandler);

    const std::string name = "obj_detection";

    DataLogger::get().createMainLog(name);
    ConfigurationManager::getInstance()->loadConfiguration();

    rclcpp::init(0, nullptr);
    RosTopicManager::getInstance(name);
    RosTopicManager::getInstance()->spinNode();
    HeartbeatPublisher heartbeat(name);

    ObjectDetectionHandler objectDetectionHandler;

    while(rclcpp::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    rclcpp::shutdown();
}
