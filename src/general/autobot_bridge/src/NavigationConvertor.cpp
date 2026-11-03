
#include "autobot_bridge/NavigationConvertor.h"
#include "autobot_common/RosTopicManager.h"

#include <eigen3/Eigen/Dense>

NavigationConvertor::NavigationConvertor(const std::string& anIncomingTopic, const std::string& anOutgoingTopic) : 
    mIncomingTopic(anIncomingTopic), mOutgoingTopic(anOutgoingTopic)
{
    RosTopicManager::getInstance()->createPublisher<ptera_msgs::msg::RobotState>(mOutgoingTopic);

    RosTopicManager::getInstance()->createSubscriber<autobot_msgs::msg::State>(mIncomingTopic, 
            std::bind(&NavigationConvertor::convert, this, std::placeholders::_1)); 

}

NavigationConvertor::~NavigationConvertor()
{

}

void NavigationConvertor::convert(const autobot_msgs::msg::State::SharedPtr& anState)
{
    ptera_msgs::msg::Vec3 pos; 
    pos.set__x(anState->position.x); 
    pos.set__y(anState->position.y); 
    pos.set__z(0.0); 

    ptera_msgs::msg::Vec3 vel; 
    vel.set__x(anState->velocity.x); 
    vel.set__y(anState->velocity.y); 
    vel.set__z(0.0); 

    ptera_msgs::msg::Euler orient; 
    orient.set__roll(0.0); 
    orient.set__pitch(0.0); 
    orient.set__yaw(anState->position.yaw); 

    // compute quaternion 
    Eigen::Quaterniond q = Eigen::AngleAxisd(0,     Eigen::Vector3d::UnitX())
                         * Eigen::AngleAxisd(0,     Eigen::Vector3d::UnitY())
                         * Eigen::AngleAxisd(anState->position.yaw,   Eigen::Vector3d::UnitZ());
    
    ptera_msgs::msg::Quaternion qm;
    qm.set__w(q.w());
    qm.set__x(q.x()); 
    qm.set__y(q.y()); 
    qm.set__z(q.z()); 

    ptera_msgs::msg::Vec3 angVel; 
    angVel.set__x(0.0); 
    angVel.set__y(0.0); 
    angVel.set__z(anState->velocity.yaw); 

    ptera_msgs::msg::RobotState state; 
    state.set__position(pos); 
    state.set__velocity(vel); 
    state.set__euler(orient); 
    state.set__quat(qm); 
    state.set__angular_velocity(angVel); 

    state.set__timestamp(anState->timestamp); 

    RosTopicManager::getInstance()->publishMessage(mOutgoingTopic, state);  
}