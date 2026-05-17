
#include "abv_bridge/ControllerStatusConvertor.h"
#include "abv_common/RosTopicManager.h"

#include "ptera_msgs/msg/controller_status.hpp"

ControllerStatusConvertor::ControllerStatusConvertor(const std::string& anIncomingTopic, const std::string& anOutgoingTopic) :
    mIncomingTopic(anIncomingTopic), mOutgoingTopic(anOutgoingTopic)
{
    RosTopicManager::getInstance()->createPublisher<ptera_msgs::msg::ControllerStatus>(mOutgoingTopic);
    RosTopicManager::getInstance()->createSubscriber<abv_msgs::msg::AbvControllerStatus>(mIncomingTopic,
            std::bind(&ControllerStatusConvertor::convert, this, std::placeholders::_1));
}

ControllerStatusConvertor::~ControllerStatusConvertor()
{

}

void ControllerStatusConvertor::convert(abv_msgs::msg::AbvControllerStatus::SharedPtr aStatus)
{
    ptera_msgs::msg::ControllerStatus status;

    switch (aStatus->arrival)
    {
    case abv_msgs::msg::AbvControllerStatus::ARRIVED:
        status.arrival = ptera_msgs::msg::ControllerStatus::ARRIVED;
        break;
    case abv_msgs::msg::AbvControllerStatus::IDLE:
        status.arrival = ptera_msgs::msg::ControllerStatus::IDLE;
        break;
    case abv_msgs::msg::AbvControllerStatus::RUNNING:
        status.arrival = ptera_msgs::msg::ControllerStatus::RUNNING;
        break;
    default:
        status.arrival = ptera_msgs::msg::ControllerStatus::RUNNING;
        break;
    }

    RosTopicManager::getInstance()->publishMessage(mOutgoingTopic, status);
}
