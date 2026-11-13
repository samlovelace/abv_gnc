#include "autobot_control/ControlLawHandler.h"
#include "autobot_control/PidControlLaw.h"

#include "autobot_common/RosTopicManager.h"
#include "autobot_config/GncConfigManager.h"
#include "autobot_msgs/msg/thruster_command.hpp"
#include "plog/Log.h"

namespace
{
    Eigen::Vector3d toEigen(const autobot_msgs::msg::Vec3& aVec)
    {
        return Eigen::Vector3d(aVec.x, aVec.y, aVec.yaw);
    }

    autobot_msgs::msg::Vec3 toMsg(const Eigen::Vector3d& aVec)
    {
        autobot_msgs::msg::Vec3 msg;
        msg.x = aVec.x();
        msg.y = aVec.y();
        msg.yaw = aVec.z();
        return msg;
    }
}

ControlLawHandler::ControlLawHandler()
{
    auto& config = GncConfigManager::getInstance()->getControlLawConfig();

    if("LQR" == config.mLawType || "lqr" == config.mLawType ||
       "MPC" == config.mLawType || "mpc" == config.mLawType)
    {
        LOGW << "ControlLaw.Law '" << config.mLawType << "' is not implemented yet, defaulting to PID";
        mControlLaw = std::make_unique<PidControlLaw>();
    }
    else
    {
        if("PID" != config.mLawType && "pid" != config.mLawType)
        {
            LOGW << "Unknown ControlLaw.Law '" << config.mLawType << "', defaulting to PID";
        }
        mControlLaw = std::make_unique<PidControlLaw>();
    }

    auto topicManager = RosTopicManager::getInstance();
    topicManager->createPublisher<autobot_msgs::msg::ThrusterCommand>("abv/thrusters/command");
    topicManager->createSubscriber<autobot_msgs::msg::ControlContext>("abv/controller/context",
                                    std::bind(&ControlLawHandler::contextCallback, this, std::placeholders::_1));
    topicManager->createSubscriber<autobot_msgs::msg::PolicyActive>("abv/policy/active",
                                    std::bind(&ControlLawHandler::policyActiveCallback, this, std::placeholders::_1));

    topicManager->spinNode();
}

ControlLawHandler::~ControlLawHandler()
{
}

void ControlLawHandler::policyActiveCallback(autobot_msgs::msg::PolicyActive::SharedPtr aMsg)
{
    bool wasActive = mIsActive;
    mIsActive = (aMsg->policy_name == mPolicyName);

    if(mIsActive && !wasActive)
    {
        mControlLaw->reset();
    }
}

void ControlLawHandler::contextCallback(autobot_msgs::msg::ControlContext::SharedPtr aCtxMsg)
{
    if(!mIsActive)
    {
        return;
    }

    ControlContext ctx;
    ctx.currentPose = toEigen(aCtxMsg->pose);
    ctx.currentVelocity = toEigen(aCtxMsg->velocity);
    ctx.goal = toEigen(aCtxMsg->goal);
    ctx.error = toEigen(aCtxMsg->error);
    ctx.goalType = (autobot_msgs::msg::ControlContext::VELOCITY == aCtxMsg->goal_type) ? GoalType::VELOCITY : GoalType::POSE;

    ActionContext actionCtx;
    if(!mControlLaw->computeAction(ctx, actionCtx))
    {
        LOGE << "Failed to compute control action";
        return;
    }

    autobot_msgs::msg::ThrusterCommand cmd;
    cmd.type = "direction";
    cmd.is_global = actionCtx.isGlobal;
    cmd.data = toMsg(actionCtx.controlInput);
    cmd.policy_name = mPolicyName;

    RosTopicManager::getInstance()->publishMessage<autobot_msgs::msg::ThrusterCommand>("abv/thrusters/command", cmd);
}
