
#include "abv_bridge/NavigationConvertor.h"
#include "common/RosTopicManager.h"

NavigationConvertor::NavigationConvertor(const std::string& anIncomingTopic, const std::string& anOutgoingTopic) : 
    mIncomingTopic(anIncomingTopic), mOutgoingTopic(anOutgoingTopic)
{
    RosTopicManager::getInstance()->createPublisher<robot_idl::msg::RobotState>(mOutgoingTopic);

    RosTopicManager::getInstance()->createSubscriber<abv_msgs::msg::AbvState>(mIncomingTopic, 
            std::bind(&NavigationConvertor::convert, this, std::placeholders::_1)); 

}

NavigationConvertor::~NavigationConvertor()
{

}

void NavigationConvertor::convert(const abv_msgs::msg::AbvState::SharedPtr& anAbvState)
{
    robot_idl::msg::Vec3 pos; 
    pos.set__x(anAbvState->position.x); 
    pos.set__y(anAbvState->position.y); 
    pos.set__z(0.0); 

    robot_idl::msg::Vec3 vel; 
    vel.set__x(anAbvState->velocity.x); 
    vel.set__y(anAbvState->velocity.y); 
    vel.set__z(0.0); 

    robot_idl::msg::Euler orient; 
    orient.set__roll(0.0); 
    orient.set__pitch(0.0); 
    orient.set__yaw(anAbvState->position.yaw); 

    robot_idl::msg::Vec3 angVel; 
    angVel.set__x(0.0); 
    angVel.set__y(0.0); 
    angVel.set__z(anAbvState->velocity.yaw); 

    robot_idl::msg::RobotState state; 
    state.set__position(pos); 
    state.set__velocity(vel); 
    state.set__euler(orient); 
    state.set__angular_velocity(angVel); 

    // TODO: compute quaternion 

    RosTopicManager::getInstance()->publishMessage(mOutgoingTopic, state);  
}