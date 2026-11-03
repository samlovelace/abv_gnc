
#include <cstdio>
#include "autobot_goal_manager/CommandHandler.h"
#include "autobot_goal_manager/StateMachine.h"
#include "autobot_goal_manager/GoalManager.h"
#include "autobot_common/DataLogger.h"
#include "autobot_common/SignalHandler.hpp"
#include "autobot_common/ConfigurationManager.h"
#include "autobot_common/HeartbeatPublisher.h"

int main()
{
    std::signal(SIGINT, signalHandler);

    DataLogger::get().createMainLog("autobot_goal_manager");
    ConfigurationManager::getInstance()->loadConfiguration();

    rclcpp::init(0, nullptr);
    RosTopicManager::getInstance("autobot_goal_manager");
    RosTopicManager::getInstance()->spinNode();
    HeartbeatPublisher heartbeat("goal_manager");

    std::shared_ptr<GoalManager> abv = std::make_shared<GoalManager>();

    std::shared_ptr<StateMachine> msm = std::make_shared<StateMachine>(abv);
    std::unique_ptr<CommandHandler> cmdHandler = std::make_unique<CommandHandler>(msm, abv);

    msm->run();

    rclcpp::shutdown();
}
