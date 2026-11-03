
#include "autobot_common/RosTopicManager.h"
#include "autobot_common/RateController.hpp"

#include "autobot_navigation/RosStatePublisher.h"

#include <thread>
#include <chrono> 
#include "plog/Log.h"

RosStatePublisher::RosStatePublisher() : mTopicName("abv/state")
{
    RosTopicManager::getInstance()->createPublisher<autobot_msgs::msg::State>(mTopicName);
}

RosStatePublisher::~RosStatePublisher()
{

}

void RosStatePublisher::publish(const State& aState,
                                 std::chrono::system_clock::time_point aMeasurementTime,
                                 bool aValid)
{
    RosTopicManager::getInstance()->publishMessage<autobot_msgs::msg::State>(
        mTopicName, convertToIdlMsg(aState, aMeasurementTime, aValid));
}

autobot_msgs::msg::State RosStatePublisher::convertToIdlMsg(const State& aStateVector,
                                                            std::chrono::system_clock::time_point aMeasurementTime,
                                                            bool aValid)
{
    autobot_msgs::msg::Vec3 position;
    autobot_msgs::msg::Vec3 velocity;

    position.x = aStateVector.x;
    position.y = aStateVector.y;
    position.yaw = aStateVector.theta;

    velocity.x = aStateVector.vx;
    velocity.y = aStateVector.vy;
    velocity.yaw = aStateVector.omega;

    autobot_msgs::msg::State state;
    state.set__position(position);
    state.set__velocity(velocity);
    state.set__valid(aValid);

    rclcpp::Time measurementTime(std::chrono::duration_cast<std::chrono::nanoseconds>(
        aMeasurementTime.time_since_epoch()).count(), RCL_SYSTEM_TIME);
    state.set__timestamp(measurementTime);

    return state;
}
