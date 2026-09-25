
#include <cstdio>
#include "abv_controller/CommandHandler.h"
#include "abv_controller/StateMachine.h"
#include "abv_controller/Vehicle.h"
#include "abv_common/DataLogger.h"
#include "abv_common/SignalHandler.hpp"
#include "abv_common/ConfigurationManager.h"
#include "abv_common/HeartbeatPublisher.h"

int main()
{
    std::signal(SIGINT, signalHandler); 

    DataLogger::get().createMainLog("abv_controller");
    ConfigurationManager::getInstance()->loadConfiguration();
    
    rclcpp::init(0, nullptr);
    RosTopicManager::getInstance("abv_controller");

    // start spinning and publishing the heartbeat immediately, before
    // constructing Vehicle - ExternalControlPolicy's constructor can block
    // indefinitely on wait_for_service() with no timeout, and the heartbeat
    // should still show this process as alive while that's in progress
    // rather than waiting for it to resolve (spinNode() is idempotent, so
    // CommandHandler's own internal spinNode() call later is a no-op)
    RosTopicManager::getInstance()->spinNode();
    HeartbeatPublisher heartbeat("controller");

    std::shared_ptr<Vehicle> abv = std::make_shared<Vehicle>();
    abv->init();

    std::shared_ptr<StateMachine> msm = std::make_shared<StateMachine>(abv);
    std::unique_ptr<CommandHandler> cmdHandler = std::make_unique<CommandHandler>(msm, abv);

    msm->run();

    rclcpp::shutdown(); 
}