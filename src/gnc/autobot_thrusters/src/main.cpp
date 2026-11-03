
#include <cstdio>
#include "autobot_common/RosTopicManager.h"
#include "autobot_common/DataLogger.h"
#include "autobot_common/SignalHandler.hpp"
#include "autobot_common/ConfigurationManager.h"
#include "autobot_common/HeartbeatPublisher.h"
#include "autobot_thrusters/ThrusterCommandHandler.h"

int main()
{
    std::signal(SIGINT, signalHandler);

    DataLogger::get().createMainLog("autobot_thrusters");
    ConfigurationManager::getInstance()->loadConfiguration();

    rclcpp::init(0, nullptr);
    RosTopicManager::getInstance("autobot_thrusters");
    RosTopicManager::getInstance()->spinNode();
    HeartbeatPublisher heartbeat("thrusters");

    ThrusterCommandHandler thrusterCommandHandler;

    while(rclcpp::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    rclcpp::shutdown();
}
