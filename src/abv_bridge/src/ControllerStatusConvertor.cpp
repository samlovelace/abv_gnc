
#include "abv_bridge/ControllerStatusConvertor.h"
#include "common/RosTopicManager.h"

#include "robot_idl/msg/controller_status.hpp"

ControllerStatusConvertor::ControllerStatusConvertor(const std::string& anIncomingTopic, const std::string& anOutgoingTopic) : 
    mIncomingTopic(anIncomingTopic), mOutgoingTopic(anOutgoingTopic)
{
    RosTopicManager::getInstance()->createPublisher<robot_idl::msg::ControllerStatus>(mOutgoingTopic); 
    RosTopicManager::getInstance()->createSubscriber<abv_msgs::msg::AbvControllerStatus>(mIncomingTopic, 
            std::bind(&ControllerStatusConvertor::convert, this, std::placeholders::_1)); 
}

ControllerStatusConvertor::~ControllerStatusConvertor()
{

}

void ControllerStatusConvertor::convert(abv_msgs::msg::AbvControllerStatus::SharedPtr aStatus)
{
    robot_idl::msg::ControllerStatus status; 

    switch (aStatus->arrival)
    {
    case abv_msgs::msg::AbvControllerStatus::ARRIVED:
        status.arrival = robot_idl::msg::ControllerStatus::ARRIVED; 
        break;
    case abv_msgs::msg::AbvControllerStatus::IDLE: 
        status.arrival = robot_idl::msg::ControllerStatus::IDLE; 
        break; 
    case abv_msgs::msg::AbvControllerStatus::RUNNING: 
        status.arrival = robot_idl::msg::ControllerStatus::RUNNING; 
        break; 
    default:
        status.arrival = robot_idl::msg::ControllerStatus::RUNNING; 
        break;
    }

    RosTopicManager::getInstance()->publishMessage(mOutgoingTopic, status); 
}
