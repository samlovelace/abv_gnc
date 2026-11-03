
#include <thread>
#include <memory>

#include "autobot_common/ConfigurationManager.h"
#include "autobot_common/RosTopicManager.h"
#include "autobot_common/RateController.hpp"
#include "autobot_simulator/VehicleSimulator.h"

int main()
{
    ConfigurationManager::getInstance()->loadConfiguration();

    rclcpp::init(0, nullptr);
    RosTopicManager::getInstance("autobot_simulator");

    std::unique_ptr<VehicleSimulator> abv = std::make_unique<VehicleSimulator>();

    abv->listen(); 
    RateController rate(50); 
    printf("##################### ABV SIM ##################################\n");

    while(true)
    {
        rate.start(); 
        abv->update(rate.getDeltaTime());
        rate.block(); 
    }

    rclcpp::shutdown(); 
}