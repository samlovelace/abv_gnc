#ifndef GRASP_PLANNER_HANDLER_H
#define GRASP_PLANNER_HANDLER_H

#include <rclcpp/rclcpp.hpp>

#include "autobot_msgs/msg/object_observation.hpp"
#include "autobot_msgs/srv/get_tracked_objects.hpp"
#include "autobot_msgs/srv/plan_grasp.hpp"

class GraspPlannerHandler
{
public:
    GraspPlannerHandler();
    ~GraspPlannerHandler();

private:
    void planGraspCallback(const std::shared_ptr<autobot_msgs::srv::PlanGrasp::Request> aRequest,
                            std::shared_ptr<autobot_msgs::srv::PlanGrasp::Response> aResponse);

    // Builds the rebot_end_link orientation: local x fixed to world -z (approach
    // straight down), local y the object's min-thickness principal axis projected
    // ⟂ to x (Gram-Schmidt - the raw axis isn't guaranteed perpendicular to -z),
    // local z completing a right-handed frame. Returns false (leaving aQuatOut
    // untouched) if both the thinnest and middle axes are too close to vertical
    // to produce a well-conditioned finger-closing direction.
    bool computeOrientation(const autobot_msgs::msg::ObjectObservation& aObservation,
                             geometry_msgs::msg::Quaternion& aQuatOut) const;

    // Minimum world-frame z among the observation's points - the point farthest
    // from the overhead camera, i.e. the bottom edge of the visible cap, which
    // approximates half the object's height (see GraspPlannerHandler.cpp for why
    // this is the min, not the max, once everything is in world frame).
    static bool minPointZ(const sensor_msgs::msg::PointCloud2& aCloud, double& aMinZOut);

    // planGraspCallback runs on RosTopicManager's single-threaded executor
    // (the one servicing mPlanGraspService itself), so it must not block that
    // same executor waiting on mGetTrackedObjectsClient's response - that would
    // deadlock, since nothing else could ever spin the response in. The client
    // lives on its own node/executor instead, spun synchronously and only for
    // the duration of each call via rclcpp::spin_until_future_complete.
    rclcpp::Node::SharedPtr mClientNode;
    rclcpp::Client<autobot_msgs::srv::GetTrackedObjects>::SharedPtr mGetTrackedObjectsClient;
    rclcpp::Service<autobot_msgs::srv::PlanGrasp>::SharedPtr mPlanGraspService;
};
#endif  // GRASP_PLANNER_HANDLER_H
