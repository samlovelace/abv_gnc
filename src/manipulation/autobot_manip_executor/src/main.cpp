#include <cstdio>
#include "autobot_common/RosTopicManager.h"
#include "autobot_common/DataLogger.h"
#include "autobot_common/SignalHandler.hpp"
#include "autobot_common/ConfigurationManager.h"
#include "autobot_common/HeartbeatPublisher.h"
#include "autobot_manip_common/ConfigManager.h"
#include "autobot_manip_executor/ManipExecutorHandler.h"

int main()
{
    std::signal(SIGINT, signalHandler);

    const std::string name = "manip_executor";

    DataLogger::get().createMainLog(name);
    ConfigurationManager::getInstance()->loadConfiguration();
    ConfigManager::getInstance()->loadConfig();

    rclcpp::init(0, nullptr);
    RosTopicManager::getInstance(name);
    RosTopicManager::getInstance()->spinNode();
    HeartbeatPublisher heartbeat(name);

    ManipExecutorHandler manipExecutorHandler;

    while(rclcpp::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    rclcpp::shutdown();
}
