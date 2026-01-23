
#include <cstdio> 
#include "common/RosTopicManager.h"
#include "common/DataLogger.h"
#include "abv_guidance/StateMachine.h"
#include "abv_guidance/ICommandSource.hpp"
#include "abv_guidance/RosCommandSource.h"

int main()
{
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