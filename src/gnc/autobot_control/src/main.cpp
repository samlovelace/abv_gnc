
#include <cstdio>
#include "autobot_common/RosTopicManager.h"
#include "autobot_common/DataLogger.h"
#include "autobot_common/SignalHandler.hpp"
#include "autobot_config/CommonConfigManager.h"
#include "autobot_config/GncConfigManager.h"
#include "autobot_common/HeartbeatPublisher.h"
#include "autobot_control/ControlLawHandler.h"

int main()
{
    std::signal(SIGINT, signalHandler);

    DataLogger::get().createMainLog("autobot_control");
    CommonConfigManager::getInstance()->loadConfiguration();
    GncConfigManager::getInstance()->loadConfiguration();

    rclcpp::init(0, nullptr);
    RosTopicManager::getInstance("autobot_control");
    RosTopicManager::getInstance()->spinNode();
    HeartbeatPublisher heartbeat("control");
    HeartbeatPublisher policyHeartbeat("control", "abv/policy/status");

    ControlLawHandler controlLawHandler;

    while(rclcpp::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    rclcpp::shutdown();
}
