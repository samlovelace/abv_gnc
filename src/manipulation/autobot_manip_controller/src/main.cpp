
#include <autobot_common/RosTopicManager.h>
#include <cstdio>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "autobot_common/DataLogger.h"
#include "autobot_config/ManipConfigManager.h"

#include "autobot_manip_controller/Manipulator.h"
#include "autobot_manip_controller/StateMachine.h"
#include "autobot_manip_controller/CommandHandler.h"

void signalHandler(int signal)
{
    RCLCPP_INFO(rclcpp::get_logger("autobot_manip_controller"), "Received signal %d", signal);
    rclcpp::shutdown();
    exit(0); 
}

int main()
{
    std::signal(SIGINT, signalHandler);

    rclcpp::init(0, nullptr);
    DataLogger::get().createMainLog("autobot_manip_controller");
    RosTopicManager::getInstance("autobot_manip_controller")->spinNode();

    ManipConfigManager::getInstance()->loadConfig();

    auto manip = std::make_shared<Manipulator>(ManipConfigManager::getInstance()->getConfig());
    auto sm = std::make_shared<StateMachine>(manip);
    CommandHandler* cm = new CommandHandler(sm, manip);

    sm->run();

    return 0;
}
