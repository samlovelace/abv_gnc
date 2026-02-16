
#include <cstdio> 
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "common/RosTopicManager.h"
#include "common/DataLogger.h"
#include "common/SignalHandler.hpp"
#include "common/ConfigurationManager.h"

#include "abv_guidance/StateMachine.h"
#include "abv_guidance/ICommandSource.hpp"
#include "abv_guidance/RosCommandSource.h"

int main()
{
    std::signal(SIGINT, signalHandler); 
    
    std::string configFilePath = ament_index_cpp::get_package_share_directory("abv_gnc") + "/configuration/config.yaml"; 
    if(!ConfigurationManager::getInstance()->loadConfiguration(configFilePath))
    {
        printf("Could not load config file at %s\n", configFilePath.c_str()); 
        return 0; 
    }

    rclcpp::init(0, nullptr);
    RosTopicManager::getInstance("abv_guidance");  
    DataLogger::get().createMainLog("abv_guidance"); 

    StateMachine sm; 

    std::vector<std::unique_ptr<ICommandSource>> sources; 
    sources.push_back(std::move(std::make_unique<RosCommandSource>(sm)));

    for(const auto& s : sources)
    {
        s->listen(); 
    }

    sm.run(); 

    rclcpp::shutdown(); 
}