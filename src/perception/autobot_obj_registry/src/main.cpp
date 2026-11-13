#include <cstdio>
#include "autobot_common/RosTopicManager.h"
#include "autobot_common/DataLogger.h"
#include "autobot_common/SignalHandler.hpp"
#include "autobot_config/CommonConfigManager.h"
#include "autobot_config/PercepConfigManager.h"
#include "autobot_common/HeartbeatPublisher.h"
#include "autobot_obj_registry/ObjectRegistryHandler.h"

int main()
{
    std::signal(SIGINT, signalHandler);

    const std::string name = "obj_registry";

    DataLogger::get().createMainLog(name);
    CommonConfigManager::getInstance()->loadConfiguration();
    PercepConfigManager::getInstance()->loadConfiguration();

    rclcpp::init(0, nullptr);
    RosTopicManager::getInstance(name);
    RosTopicManager::getInstance()->spinNode();
    HeartbeatPublisher heartbeat(name);

    ObjectRegistryHandler objectRegistryHandler;

    while(rclcpp::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    rclcpp::shutdown();
}
