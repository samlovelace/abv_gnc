
#include <cstdio>
#include "autobot_common/RosTopicManager.h"
#include "autobot_common/DataLogger.h"
#include "autobot_common/SignalHandler.hpp"
#include "autobot_config/CommonConfigManager.h"
#include "autobot_common/HeartbeatPublisher.h"
#include "autobot_policy_registry/PolicyRegistry.h"

int main()
{
    std::signal(SIGINT, signalHandler);

    DataLogger::get().createMainLog("autobot_policy_registry");
    CommonConfigManager::getInstance()->loadConfiguration();

    rclcpp::init(0, nullptr);
    RosTopicManager::getInstance("autobot_policy_registry");
    RosTopicManager::getInstance()->spinNode();
    HeartbeatPublisher heartbeat("policy_registry");

    PolicyRegistry policyRegistry;

    while(rclcpp::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    rclcpp::shutdown();
}
