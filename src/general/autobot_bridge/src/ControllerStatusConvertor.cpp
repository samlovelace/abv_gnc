
#include "autobot_bridge/ControllerStatusConvertor.h"
#include "autobot_common/RosTopicManager.h"

#include "ptera_msgs/msg/controller_status.hpp"

ControllerStatusConvertor::ControllerStatusConvertor(const std::string& anIncomingTopic, const std::string& anOutgoingTopic) :
    mIncomingTopic(anIncomingTopic), mOutgoingTopic(anOutgoingTopic)
{
    RosTopicManager::getInstance()->createPublisher<ptera_msgs::msg::ControllerStatus>(mOutgoingTopic);
    RosTopicManager::getInstance()->createSubscriber<autobot_msgs::msg::ControllerStatus>(mIncomingTopic,
            std::bind(&ControllerStatusConvertor::convert, this, std::placeholders::_1));
}

ControllerStatusConvertor::~ControllerStatusConvertor()
{

}

void ControllerStatusConvertor::convert(autobot_msgs::msg::ControllerStatus::SharedPtr aStatus)
{
    ptera_msgs::msg::ControllerStatus status;

    switch (aStatus->arrival)
    {
    case autobot_msgs::msg::ControllerStatus::ARRIVED:
        status.arrival = ptera_msgs::msg::ControllerStatus::ARRIVED;
        break;
    case autobot_msgs::msg::ControllerStatus::IDLE:
        status.arrival = ptera_msgs::msg::ControllerStatus::IDLE;
        break;
    case autobot_msgs::msg::ControllerStatus::RUNNING:
        status.arrival = ptera_msgs::msg::ControllerStatus::RUNNING;
        break;
    default:
        status.arrival = ptera_msgs::msg::ControllerStatus::RUNNING;
        break;
    }

    RosTopicManager::getInstance()->publishMessage(mOutgoingTopic, status);
}
