// ManipPlannerHandler.h (via MotionPlanner.h) must be included before any
// header that pulls plog/Log.h (SignalHandler.hpp, ConfigurationManager.h
// below) - plog's LOG_DEBUG/LOG_WARN/etc. macros collide with OMPL's own
// identically-named enum members in ompl/util/Console.h, corrupting that
// enum's declaration if plog's macros are already defined when it's parsed.
#include "autobot_manip_planner/ManipPlannerHandler.h"

#include <cstdio>
#include "autobot_common/RosTopicManager.h"
#include "autobot_common/DataLogger.h"
#include "autobot_common/SignalHandler.hpp"
#include "autobot_common/ConfigurationManager.h"
#include "autobot_common/HeartbeatPublisher.h"
#include "autobot_manip_common/ConfigManager.h"

int main()
{
    std::signal(SIGINT, signalHandler);

    const std::string name = "manip_planner";

    DataLogger::get().createMainLog(name);
    ConfigurationManager::getInstance()->loadConfiguration();
    ConfigManager::getInstance()->loadConfig();

    rclcpp::init(0, nullptr);
    RosTopicManager::getInstance(name);
    RosTopicManager::getInstance()->spinNode();
    HeartbeatPublisher heartbeat(name);

    ManipPlannerHandler manipPlannerHandler;

    while(rclcpp::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    rclcpp::shutdown();
}
