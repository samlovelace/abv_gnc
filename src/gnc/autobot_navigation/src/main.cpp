
#include <cstdio>
#include "autobot_common/RosTopicManager.h"
#include "autobot_common/DataLogger.h"
#include "autobot_common/SignalHandler.hpp"
#include "autobot_config/CommonConfigManager.h"
#include "autobot_config/GncConfigManager.h"
#include "autobot_common/HeartbeatPublisher.h"
#include "autobot_navigation/VehicleStateTracker.h"
#include "autobot_navigation/RosStatePublisher.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

int main()
{
    std::signal(SIGINT, signalHandler); 

    // instantiate singletons 
    DataLogger::get().createMainLog("autobot_navigation");
    CommonConfigManager::getInstance()->loadConfiguration();
    GncConfigManager::getInstance()->loadConfiguration();
    
    rclcpp::init(0, nullptr);
    RosTopicManager::getInstance("autobot_navigation"); 
    RosTopicManager::getInstance()->spinNode(); 

    VehicleStateTracker stateTracker;

    HeartbeatPublisher heartbeat("navigation");

    while(true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 
    }

    rclcpp::shutdown(); 
}