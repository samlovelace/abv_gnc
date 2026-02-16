
#include "common/RosTopicManager.h"
#include "common/RateController.hpp"

#include "abv_navigation/RosStatePublisher.h"

#include <thread>
#include <chrono> 
#include "plog/Log.h"

RosStatePublisher::RosStatePublisher() : mTopicName("abv/state")
{
    RosTopicManager::getInstance()->createPublisher<abv_msgs::msg::AbvState>(mTopicName);
}

RosStatePublisher::~RosStatePublisher()
{

}

void RosStatePublisher::publish(const AbvState& aState)
{
    RosTopicManager::getInstance()->publishMessage<abv_msgs::msg::AbvState>(mTopicName, convertToIdlMsg(aState)); 
}

abv_msgs::msg::AbvState RosStatePublisher::convertToIdlMsg(const AbvState& aStateVector)
{
    abv_msgs::msg::Vec3 position; 
    abv_msgs::msg::Vec3 velocity;

    position.x = aStateVector.x; 
    position.y = aStateVector.y; 
    position.z = 0.0; 

    velocity.x = aStateVector.vx; 
    velocity.y = aStateVector.vy; 
    velocity.z = 0.0; 
    
    abv_msgs::msg::Vec3 orientation; 
    orientation.x = 0.0;
    orientation.y = 0.0; 
    orientation.z = aStateVector.theta;  

    abv_msgs::msg::Vec3 ang_vel; 
    ang_vel.x = 0.0;
    ang_vel.y = 0.0; 
    ang_vel.z = aStateVector.omega; 

    abv_msgs::msg::AbvState state; 
    state.set__position(position); 
    state.set__velocity(velocity); 
    state.set__orientation(orientation); 
    state.set__ang_vel(ang_vel); 

    return state; 
}
