#ifndef INPUTHANDLER_H
#define INPUTHANDLER_H

#include "autobot_commander/Colors.hpp"
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include "autobot_msgs/srv/plan_grasp.hpp"
#include "autobot_msgs/msg/joint_position_waypoint.hpp"
#include "autobot_msgs/msg/execute_pick_command.hpp"
#include "autobot_msgs/msg/execution_status.hpp"
#include <mutex>
#include <condition_variable>

class InputHandler
{
public:
    InputHandler();
    ~InputHandler();

    void handle(const std::string& anInput);

private:

    void handleGrasp();

    // Publishes a gripper waypoint on arm/gripper_position_waypoint, which
    // autobot_manip_controller's CommandHandler passes straight through to
    // the hardware (no WaypointExecutor/arrival tracking for the gripper).
    void handleGripper(bool anOpen);

    // Publishes an ExecutePickCommand on manip/execute_pick_command, then
    // waits (on a lazily-created subscription to manip/execution_status) for
    // the terminal SUCCEEDED/FAILED status and prints it.
    void handleExecute();
    void executionStatusCallback(const autobot_msgs::msg::ExecutionStatus::SharedPtr aMsg);

    // T_M_O = T_M_C * T_C_W * T_W_O - transforms a grasp pose given in the
    // world frame (what the grasp planner returns, since it's built from a
    // point cloud already expressed in world frame) into the manip base frame.
    geometry_msgs::msg::Pose transformWorldPoseToManipFrame(const geometry_msgs::msg::Pose& aPoseInWorld) const;

    std::string mPackagePath;

    // Lazily created on first "grasp" command - must not be created before
    // rclcpp::init() (CommanderComms's constructor), which hasn't run yet
    // when InputHandler itself is constructed.
    rclcpp::Client<autobot_msgs::srv::PlanGrasp>::SharedPtr mPlanGraspClient;

    // Lazily subscribed on first "execute" command, same reasoning as above.
    bool mExecutionStatusSubscribed{false};
    std::mutex mExecutionStatusMutex;
    std::condition_variable mExecutionStatusCv;
    autobot_msgs::msg::ExecutionStatus mLatestExecutionStatus;
    bool mHasExecutionStatus{false};
};
#endif //INPUTHANDLER_H