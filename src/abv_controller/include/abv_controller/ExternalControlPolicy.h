
#ifndef EXTERNAL_CONTROL_POLICY_H
#define EXTERNAL_CONTROL_POLICY_H

#include "abv_controller/IControlPolicy.hpp"
#include "abv_msgs/srv/abv_control_action.hpp"
#include <rclcpp/rclcpp.hpp>

class ExternalControlPolicy : public IControlPolicy
{
public:
    explicit ExternalControlPolicy();
    ~ExternalControlPolicy() override = default;

    Eigen::Vector3d computeAction(const ControlContext& ctx) override;

private:
    rclcpp::Node::SharedPtr mNode;
    rclcpp::Client<abv_msgs::srv::AbvControlAction>::SharedPtr mClient;
};

#endif