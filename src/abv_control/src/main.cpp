
#include <cstdio>
#include "abv_controller/CommandHandler.h"
#include "abv_controller/StateMachine.h"
#include "abv_controller/Vehicle.h"
#include "common/DataLogger.h"
#include "common/SignalHandler.hpp"
#include "common/ConfigurationManager.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

int main()
{
    std::signal(SIGINT, signalHandler); 

    DataLogger::get().createMainLog("abv_controller");

    std::string configFilePath = ament_index_cpp::get_package_share_directory("abv_gnc") + "/configuration/config.yaml"; 
    if(!ConfigurationManager::getInstance()->loadConfiguration(configFilePath))
    {
        printf("Could not load config file at %s\n", configFilePath.c_str()); 
        return 0; 
    }
    
    rclcpp::init(0, nullptr);
    RosTopicManager::getInstance("abv_controller"); 
    
    std::shared_ptr<Vehicle> abv = std::make_shared<Vehicle>(); 
    abv->init(); 

    std::shared_ptr<StateMachine> msm = std::make_shared<StateMachine>(abv);
    std::unique_ptr<CommandHandler> cmdHandler = std::make_unique<CommandHandler>(msm, abv); 

    msm->run(); 

    rclcpp::shutdown(); 
}