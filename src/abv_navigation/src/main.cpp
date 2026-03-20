
#include <cstdio>
#include "abv_common/RosTopicManager.h"
#include "abv_common/DataLogger.h"
#include "abv_common/SignalHandler.hpp"
#include "abv_common/ConfigurationManager.h"
#include "abv_navigation/VehicleStateTracker.h"
#include "abv_navigation/RosStatePublisher.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

int main()
{
    std::signal(SIGINT, signalHandler); 

    // instantiate singletons 
    DataLogger::get().createMainLog("abv_navigation");
    ConfigurationManager::getInstance()->loadConfiguration();
    
    rclcpp::init(0, nullptr);
    RosTopicManager::getInstance("abv_navigation"); 
    RosTopicManager::getInstance()->spinNode(); 

    VehicleStateTracker stateTracker;
    
    while(true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 
    }

    rclcpp::shutdown(); 
}