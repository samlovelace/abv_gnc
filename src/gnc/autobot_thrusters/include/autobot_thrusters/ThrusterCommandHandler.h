#ifndef THRUSTERCOMMANDHANDLER_H
#define THRUSTERCOMMANDHANDLER_H

#include <memory>
#include <eigen3/Eigen/Dense>

#include "autobot_common/RosNavigationListener.h"
#include "autobot_msgs/msg/thruster_command.hpp"
#include "autobot_msgs/msg/policy_active.hpp"
#include "autobot_thrusters/ThrusterCommander.h"

/**
 * @brief ThrusterCommandHandler subscribes to abv/thrusters/command (from
 *        whichever of autobot_goal_manager/autobot_control/autobot_rl-cpp is currently
 *        publishing) and abv/state (for global->body frame conversion),
 *        and dispatches to ThrusterCommander.
 */
class ThrusterCommandHandler
{
public:
    ThrusterCommandHandler();
    ~ThrusterCommandHandler();

private:
    void commandCallback(autobot_msgs::msg::ThrusterCommand::SharedPtr aCmdMsg);
    void policyActiveCallback(autobot_msgs::msg::PolicyActive::SharedPtr aMsg);
    Eigen::Vector3d convertToBodyFrame(Eigen::Vector3d aVectorGlobal);
    void publishStatus();

private:
    std::shared_ptr<ThrusterCommander> mThrusterCommander;
    std::shared_ptr<RosNavigationListener> mNavManager;

    // Updated from abv/policy/active; gates commandCallback so only the
    // currently active policy's ThrusterCommand (non-empty policy_name)
    // reaches ThrusterCommander - a last line of defense just before
    // actuation, independent of autobot_policy_registry's own arbitration.
    // autobot_goal_manager's direct writes carry an empty policy_name and always
    // pass regardless of this value.
    std::string mActivePolicy;
};
#endif // THRUSTERCOMMANDHANDLER_H
