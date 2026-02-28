
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
    abv_msgs::msg::AbvVec3 position; 
    abv_msgs::msg::AbvVec3 velocity;

    position.x = aStateVector.x; 
    position.y = aStateVector.y; 
    position.yaw = aStateVector.theta; 

    velocity.x = aStateVector.vx; 
    velocity.y = aStateVector.vy; 
    velocity.yaw = aStateVector.omega; 

    abv_msgs::msg::AbvState state; 
    state.set__position(position); 
    state.set__velocity(velocity); 

    return state; 
}
