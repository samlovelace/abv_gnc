#ifndef THRUSTERCOMMANDHANDLER_H
#define THRUSTERCOMMANDHANDLER_H

#include <memory>
#include <eigen3/Eigen/Dense>

#include "abv_common/RosNavigationListener.h"
#include "abv_msgs/msg/abv_thruster_command.hpp"
#include "abv_thrusters/ThrusterCommander.h"

/**
 * @brief ThrusterCommandHandler subscribes to abv/thrusters/command (from
 *        whichever of abv_goal_manager/abv_control/abv_rl-cpp is currently
 *        publishing) and abv/state (for global->body frame conversion),
 *        and dispatches to ThrusterCommander.
 */
class ThrusterCommandHandler
{
public:
    ThrusterCommandHandler();
    ~ThrusterCommandHandler();

private:
    void commandCallback(abv_msgs::msg::AbvThrusterCommand::SharedPtr aCmdMsg);
    Eigen::Vector3d convertToBodyFrame(Eigen::Vector3d aVectorGlobal);
    void publishStatus();

private:
    std::shared_ptr<ThrusterCommander> mThrusterCommander;
    std::shared_ptr<RosNavigationListener> mNavManager;
};
#endif // THRUSTERCOMMANDHANDLER_H
