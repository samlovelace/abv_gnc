
#include <thread>
#include <memory>

#include "abv_common/ConfigurationManager.h"
#include "abv_common/RosTopicManager.h"
#include "abv_common/RateController.hpp"
#include "abv_simulator/VehicleSimulator.h"

int main()
{
    ConfigurationManager::getInstance()->loadConfiguration();

    rclcpp::init(0, nullptr);
    RosTopicManager::getInstance("abv_simulator");

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