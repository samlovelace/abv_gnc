
#include <cstdio>
#include "common/RosTopicManager.h"
#include "common/DataLogger.h"
#include "common/SignalHandler.hpp"
#include "common/ConfigurationManager.h"
#include "abv_navigation/VehicleStateTracker.h"
#include "abv_navigation/RosStatePublisher.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

int main()
{
    std::signal(SIGINT, signalHandler); 

    // instantiate singletons 
    DataLogger::get().createMainLog("abv_navigation");

    std::string configFilePath = ament_index_cpp::get_package_share_directory("abv_gnc") + "/configuration/config.yaml"; 
    if(!ConfigurationManager::getInstance()->loadConfiguration(configFilePath))
    {
        printf("Could not load config file at %s\n", configFilePath.c_str()); 
        return 0; 
    }
    
    rclcpp::init(0, nullptr);
    RosTopicManager::getInstance("abv_navigation"); 
    RosTopicManager::getInstance()->spinNode(); 

    VehicleStateTracker stateTracker("abv");

    while(true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 
    }

    rclcpp::shutdown(); 
}