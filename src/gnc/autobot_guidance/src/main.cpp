
#include <cstdio> 
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "autobot_common/RosTopicManager.h"
#include "autobot_common/DataLogger.h"
#include "autobot_common/SignalHandler.hpp"
#include "autobot_common/ConfigurationManager.h"
#include "autobot_common/HeartbeatPublisher.h"

#include "autobot_guidance/StateMachine.h"
#include "autobot_guidance/ICommandSource.hpp"
#include "autobot_guidance/RosCommandSource.h"

int main()
{
    std::signal(SIGINT, signalHandler); 
    
    DataLogger::get().createMainLog("autobot_guidance"); 
    ConfigurationManager::getInstance()->loadConfiguration(); 

    rclcpp::init(0, nullptr);
    RosTopicManager::getInstance("autobot_guidance");  

    StateMachine sm; 

    std::vector<std::unique_ptr<ICommandSource>> sources; 
    sources.push_back(std::move(std::make_unique<RosCommandSource>(sm)));

    for(const auto& s : sources)
    {
        s->listen();
    }

    HeartbeatPublisher heartbeat("guidance");

    sm.run();

    rclcpp::shutdown(); 
}