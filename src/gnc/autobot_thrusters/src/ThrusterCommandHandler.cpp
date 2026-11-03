#include "autobot_thrusters/ThrusterCommandHandler.h"
#include "autobot_common/RosTopicManager.h"
#include "autobot_msgs/msg/thruster_status.hpp"
#include "plog/Log.h"

ThrusterCommandHandler::ThrusterCommandHandler() :
    mThrusterCommander(std::make_shared<ThrusterCommander>()),
    mNavManager(std::make_shared<RosNavigationListener>())
{
    auto topicManager = RosTopicManager::getInstance();
    topicManager->createPublisher<autobot_msgs::msg::ThrusterStatus>("abv/thrusters/status");
    topicManager->createSubscriber<autobot_msgs::msg::ThrusterCommand>("abv/thrusters/command",
                                    std::bind(&ThrusterCommandHandler::commandCallback, this, std::placeholders::_1));
    topicManager->createSubscriber<autobot_msgs::msg::PolicyActive>("abv/policy/active",
                                    std::bind(&ThrusterCommandHandler::policyActiveCallback, this, std::placeholders::_1));

    topicManager->spinNode();
}

ThrusterCommandHandler::~ThrusterCommandHandler()
{
}

void ThrusterCommandHandler::policyActiveCallback(autobot_msgs::msg::PolicyActive::SharedPtr aMsg)
{
    mActivePolicy = aMsg->policy_name;
}

void ThrusterCommandHandler::commandCallback(autobot_msgs::msg::ThrusterCommand::SharedPtr aCmdMsg)
{
    if(!aCmdMsg->policy_name.empty() && aCmdMsg->policy_name != mActivePolicy)
    {
        LOGV << "Dropping thruster command from inactive policy '" << aCmdMsg->policy_name << "' (active: '" << mActivePolicy << "')";
        return;
    }

    if("thruster" == aCmdMsg->type)
    {
        mThrusterCommander->commandThrusters(aCmdMsg->thrusters);
        publishStatus();
        return;
    }

    if("direction" != aCmdMsg->type)
    {
        LOGW << "Unsupported ThrusterCommand type: " << aCmdMsg->type;
        return;
    }

    Eigen::Vector3d controlInput(aCmdMsg->data.x, aCmdMsg->data.y, aCmdMsg->data.yaw);

    if(aCmdMsg->is_global)
    {
        controlInput = convertToBodyFrame(controlInput);
    }

    mThrusterCommander->command(controlInput);
    publishStatus();
}

void ThrusterCommandHandler::publishStatus()
{
    autobot_msgs::msg::ThrusterStatus status;
    status.thrusters = mThrusterCommander->getThrusterCommand();

    Eigen::Vector3d applied = mThrusterCommander->getAppliedThrustVector();
    status.fx = applied.x();
    status.fy = applied.y();
    status.tz = applied.z();

    RosTopicManager::getInstance()->publishMessage<autobot_msgs::msg::ThrusterStatus>("abv/thrusters/status", status);
}

Eigen::Vector3d ThrusterCommandHandler::convertToBodyFrame(Eigen::Vector3d aVectorGlobal)
{
    auto state = mNavManager->getCurrentState();

    // yaw is element 2
    double yaw = state[2];

    // invert rotation relative to global
    Eigen::Matrix3d Rz;
    Rz << cos(yaw), sin(yaw), 0,
         -sin(yaw), cos(yaw), 0,
             0,        0,     1;

    return Rz * aVectorGlobal;
}
