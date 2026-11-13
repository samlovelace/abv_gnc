
#include "autobot_common/HeartbeatPublisher.h"
#include "autobot_common/RosTopicManager.h"
#include "autobot_config/CommonConfigManager.h"
#include "autobot_msgs/msg/heartbeat.hpp"

HeartbeatPublisher::HeartbeatPublisher(const std::string& aNodeName, const std::string& aTopic) : mNodeName(aNodeName), mTopic(aTopic)
{
    RosTopicManager::getInstance()->createPublisher<autobot_msgs::msg::Heartbeat>(mTopic);

    double rateHz = CommonConfigManager::getInstance()->getHeartbeatConfig().mRate;
    auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / rateHz));

    mTimer = RosTopicManager::getInstance()->create_wall_timer(period, std::bind(&HeartbeatPublisher::tick, this));
}

HeartbeatPublisher::~HeartbeatPublisher()
{
}

void HeartbeatPublisher::tick()
{
    autobot_msgs::msg::Heartbeat msg;
    msg.set__node_name(mNodeName);
    RosTopicManager::getInstance()->publishMessage(mTopic, msg);
}
