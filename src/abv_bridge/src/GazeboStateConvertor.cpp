
#include "abv_bridge/GazeboStateConvertor.h"
#include "abv_common/RosTopicManager.h"

GazeboStateConvertor::GazeboStateConvertor(const std::string& anIncomingTopic, const std::string& anOutgoingTopic) :
    mIncomingTopic(anIncomingTopic), mOutgoingTopic(anOutgoingTopic)
{
    RosTopicManager::getInstance()->createPublisher<abv_msgs::msg::AbvState>(mOutgoingTopic);

    RosTopicManager::getInstance()->createSubscriber<ptera_msgs::msg::RobotState>(mIncomingTopic,
            std::bind(&GazeboStateConvertor::convert, this, std::placeholders::_1));

}

GazeboStateConvertor::~GazeboStateConvertor()
{

}

void GazeboStateConvertor::convert(const ptera_msgs::msg::RobotState::SharedPtr& aRobotState)
{
    abv_msgs::msg::AbvVec3 position;
    position.x = aRobotState->position.x;
    position.y = aRobotState->position.y;
    position.yaw = aRobotState->euler.yaw;

    abv_msgs::msg::AbvVec3 velocity;
    velocity.x = aRobotState->velocity.x;
    velocity.y = aRobotState->velocity.y;
    velocity.yaw = aRobotState->angular_velocity.z;

    abv_msgs::msg::AbvState state;
    state.set__position(position);
    state.set__velocity(velocity);
    state.set__valid(true);
    state.set__timestamp(aRobotState->timestamp);

    RosTopicManager::getInstance()->publishMessage(mOutgoingTopic, state);
}
