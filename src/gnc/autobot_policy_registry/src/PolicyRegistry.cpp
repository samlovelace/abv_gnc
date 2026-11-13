#include "autobot_policy_registry/PolicyRegistry.h"

#include "autobot_common/RosTopicManager.h"
#include "autobot_config/CommonConfigManager.h"
#include "autobot_msgs/msg/policy_active.hpp"
#include "autobot_msgs/msg/policy_entry.hpp"
#include "autobot_msgs/msg/policy_registry.hpp"
#include "autobot_msgs/msg/controller_command.hpp"
#include "plog/Log.h"

PolicyRegistry::PolicyRegistry()
{
    auto topicManager = RosTopicManager::getInstance();

    topicManager->createPublisher<autobot_msgs::msg::PolicyActive>("abv/policy/active");
    topicManager->createPublisher<autobot_msgs::msg::PolicyRegistry>("abv/policy/registry");
    topicManager->createPublisher<autobot_msgs::msg::ControllerCommand>("abv/controller/command");

    topicManager->createSubscriber<autobot_msgs::msg::Heartbeat>("abv/policy/status",
                                    std::bind(&PolicyRegistry::policyStatusCallback, this, std::placeholders::_1));

    mSwitchService = topicManager->create_service<autobot_msgs::srv::PolicySwitch>("abv/policy/switch",
                                    std::bind(&PolicyRegistry::switchCallback, this, std::placeholders::_1, std::placeholders::_2));

    double rateHz = CommonConfigManager::getInstance()->getHeartbeatConfig().mRate;
    auto period = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(1.0 / rateHz));
    mTimer = topicManager->create_wall_timer(period, std::bind(&PolicyRegistry::tick, this));

    topicManager->spinNode();
}

PolicyRegistry::~PolicyRegistry()
{
}

void PolicyRegistry::policyStatusCallback(autobot_msgs::msg::Heartbeat::SharedPtr aMsg)
{
    bool isNewPolicy = (mPolicies.find(aMsg->node_name) == mPolicies.end());

    auto& record = mPolicies[aMsg->node_name];
    record.mLastSeen = std::chrono::steady_clock::now();
    record.mAlive = true;

    if(isNewPolicy)
    {
        LOGD << "Policy '" << aMsg->node_name << "' registered";
    }
}

void PolicyRegistry::switchCallback(const std::shared_ptr<autobot_msgs::srv::PolicySwitch::Request> aRequest,
                                     std::shared_ptr<autobot_msgs::srv::PolicySwitch::Response> aResponse)
{
    if(!isAlive(aRequest->policy_name))
    {
        aResponse->success = false;
        aResponse->message = "Policy '" + aRequest->policy_name + "' is not registered or not alive";
        LOGW << aResponse->message;
        return;
    }

    if(aRequest->policy_name == mActivePolicy)
    {
        aResponse->success = true;
        aResponse->message = "Policy '" + aRequest->policy_name + "' is already active";
        return;
    }

    LOGD << "Switching active policy from '" << mActivePolicy << "' to '" << aRequest->policy_name << "'";
    mActivePolicy = aRequest->policy_name;

    publishActive();
    publishClearGoal();

    aResponse->success = true;
    aResponse->message = "Switched to '" + aRequest->policy_name + "'";
}

void PolicyRegistry::tick()
{
    double staleAfter = CommonConfigManager::getInstance()->getHeartbeatConfig().mStaleAfter;
    auto now = std::chrono::steady_clock::now();

    for(auto& [name, record] : mPolicies)
    {
        bool nowAlive = std::chrono::duration<double>(now - record.mLastSeen).count() < staleAfter;
        if(nowAlive != record.mAlive)
        {
            record.mAlive = nowAlive;
            LOGD << "Policy '" << name << "' is now " << (nowAlive ? "alive" : "stale");
        }
    }

    // re-broadcast every tick (heartbeat-style), not just on change, so a
    // GUI/subscriber that connects after a policy already registered still
    // sees it - abv/policy/registry and abv/policy/active carry no QoS
    // durability, same as Heartbeat elsewhere in this codebase.
    publishActive();
    publishRegistry();
}

bool PolicyRegistry::isAlive(const std::string& aPolicyName) const
{
    auto it = mPolicies.find(aPolicyName);
    return it != mPolicies.end() && it->second.mAlive;
}

void PolicyRegistry::publishActive()
{
    autobot_msgs::msg::PolicyActive msg;
    msg.set__policy_name(mActivePolicy);
    RosTopicManager::getInstance()->publishMessage("abv/policy/active", msg);
}

void PolicyRegistry::publishRegistry()
{
    autobot_msgs::msg::PolicyRegistry msg;

    for(const auto& [name, record] : mPolicies)
    {
        autobot_msgs::msg::PolicyEntry entry;
        entry.set__policy_name(name);
        entry.set__alive(record.mAlive);
        msg.policies.push_back(entry);
    }

    RosTopicManager::getInstance()->publishMessage("abv/policy/registry", msg);
}

void PolicyRegistry::publishClearGoal()
{
    autobot_msgs::msg::ControllerCommand cmd;
    cmd.set__type("clear");
    RosTopicManager::getInstance()->publishMessage("abv/controller/command", cmd);
}
