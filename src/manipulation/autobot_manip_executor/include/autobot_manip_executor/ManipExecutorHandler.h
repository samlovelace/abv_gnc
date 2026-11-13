#ifndef MANIP_EXECUTOR_HANDLER_H
#define MANIP_EXECUTOR_HANDLER_H

#include <rclcpp/rclcpp.hpp>

#include "autobot_config/ManipConfigManager.h"
#include "autobot_manip_executor/PickExecutorStateMachine.h"
#include "autobot_msgs/msg/execute_pick_command.hpp"
#include "autobot_msgs/msg/manipulator_state.hpp"
#include "autobot_msgs/msg/execution_status.hpp"
#include "autobot_msgs/srv/plan_pick.hpp"

class ManipExecutorHandler
{
public:
    ManipExecutorHandler();
    ~ManipExecutorHandler();

private:
    void executePickCommandCallback(const autobot_msgs::msg::ExecutePickCommand::SharedPtr aMsg);
    void armStateCallback(const autobot_msgs::msg::ManipulatorState::SharedPtr aMsg);
    void tick();

    void publishStatus();

    ManipConfigManager::Config mConfig;
    PickExecutorStateMachine mStateMachine;

    // Separate client node for the one-shot, bounded manip/plan_pick call -
    // see GraspPlannerHandler's mClientNode for why this can't be the same
    // node serving the subscriptions/timer below.
    rclcpp::Node::SharedPtr mClientNode;
    rclcpp::Client<autobot_msgs::srv::PlanPick>::SharedPtr mPlanPickClient;

    rclcpp::TimerBase::SharedPtr mTickTimer;
};
#endif  // MANIP_EXECUTOR_HANDLER_H
