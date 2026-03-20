
#include <cstdio> 
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "abv_common/RosTopicManager.h"
#include "abv_common/DataLogger.h"
#include "abv_common/SignalHandler.hpp"
#include "abv_common/ConfigurationManager.h"

#include "abv_guidance/StateMachine.h"
#include "abv_guidance/ICommandSource.hpp"
#include "abv_guidance/RosCommandSource.h"

int main()
{
    std::signal(SIGINT, signalHandler); 
    
    DataLogger::get().createMainLog("abv_guidance"); 
    ConfigurationManager::getInstance()->loadConfiguration(); 

    rclcpp::init(0, nullptr);
    RosTopicManager::getInstance("abv_guidance");  

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