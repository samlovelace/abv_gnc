#ifndef MANIP_PLANNER_HANDLER_H
#define MANIP_PLANNER_HANDLER_H

#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include "autobot_msgs/msg/manip_hardware_state.hpp"
#include "autobot_msgs/msg/pick_plan.hpp"
#include "autobot_msgs/srv/plan_grasp.hpp"
#include "autobot_msgs/srv/plan_pick.hpp"
#include "autobot_config/ManipConfigManager.h"
#include "autobot_manip_common/KinematicsHandler.h"
#include "autobot_manip_planner/RobotCollisionModel.h"
#include "autobot_manip_planner/CollisionChecker.h"
#include "autobot_manip_planner/MotionPlanner.h"

class ManipPlannerHandler
{
public:
    ManipPlannerHandler();
    ~ManipPlannerHandler();

private:
    void planPickCallback(const std::shared_ptr<autobot_msgs::srv::PlanPick::Request> aRequest,
                           std::shared_ptr<autobot_msgs::srv::PlanPick::Response> aResponse);

    void hardwareStateCallback(const autobot_msgs::msg::ManipHardwareState::SharedPtr aMsg);

    // Grasp pose's rotation column 0 = approach direction, in world frame
    // (see GraspPlannerHandler::computeOrientation's convention). Backs the
    // position off by mConfig.planner.approachDistanceM along that column;
    // orientation is unchanged.
    static geometry_msgs::msg::PoseStamped computePreGraspPose(const geometry_msgs::msg::PoseStamped& aGraspPose,
                                                                 double aApproachDistanceM);

    static KDL::Frame poseToFrame(const geometry_msgs::msg::Pose& aPose);

    // Builds the final PickPlan: OMPL joint path waypoints, then the four
    // task-space stages (pre_grasp/grasp/toggle/lift) with gripper open/
    // closed positions from mConfig.gripper.
    autobot_msgs::msg::PickPlan assemblePlan(const std::vector<KDL::JntArray>& aJointPath,
                                              const geometry_msgs::msg::PoseStamped& aPreGraspPose,
                                              const geometry_msgs::msg::PoseStamped& aGraspPose) const;

    ManipConfigManager::Config mConfig;
    KinematicsHandler mKinematics;

    RobotCollisionModel mCollisionModel;
    CollisionChecker mCollisionChecker;
    MotionPlanner mMotionPlanner;

    // Separate client node for synchronous manip/plan_grasp calls - this
    // node's own service callback runs on RosTopicManager's single-threaded
    // executor, so a bare future.get() there would deadlock. Mirrors
    // GraspPlannerHandler's mClientNode pattern exactly.
    rclcpp::Node::SharedPtr mClientNode;
    rclcpp::Client<autobot_msgs::srv::PlanGrasp>::SharedPtr mPlanGraspClient;

    rclcpp::Service<autobot_msgs::srv::PlanPick>::SharedPtr mPlanPickService;

    // Latest arm/hardware/state sample - guarded defensively even though the
    // subscriber callback and planPickCallback currently both run on
    // RosTopicManager's single-threaded executor.
    std::mutex mStateMutex;
    autobot_msgs::msg::ManipHardwareState::SharedPtr mLatestState;
};
#endif  // MANIP_PLANNER_HANDLER_H
