
#include <plog/Log.h>

#include "abv_common/Utils.hpp"
#include "abv_controller/ExternalControlPolicy.h"

ExternalControlPolicy::ExternalControlPolicy()
{
    mNode = rclcpp::Node::make_shared("external_control_policy_node");
    mClient = mNode->create_client<abv_msgs::srv::AbvControlAction>("abv/control_action");

    LOGD << "Waiting for external control service...";
    mClient->wait_for_service();
    LOGD << "External control service ready";
}

Eigen::Vector3d ExternalControlPolicy::computeAction(const ControlContext& ctx)
{
    auto request = std::make_shared<abv_msgs::srv::AbvControlAction::Request>();
    
    request->pose  = utils::convertToAbvVec3(ctx.currentPose);
    request->vel   = utils::convertToAbvVec3(ctx.currentVelocity);
    request->goal  = utils::convertToAbvVec3(ctx.goal);
    request->error = utils::convertToAbvVec3(ctx.error);

    while (!mClient->wait_for_service(std::chrono::seconds(1))) 
    {
        if (!rclcpp::ok()) 
        {
            RCLCPP_ERROR(mNode->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return Eigen::Vector3d(0, 0, 0);
        }

        RCLCPP_INFO(mNode->get_logger(), "Service not available, waiting again...");
    }

    auto result_future = mClient->async_send_request(request);
    if (rclcpp::spin_until_future_complete(mNode, result_future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = result_future.get();
        return utils::convertToEigenVec3(response->action);
    } 
    else 
    {
        RCLCPP_ERROR(mNode->get_logger(), "Failed to call service abv/control_action");
        return Eigen::Vector3d(0, 0, 0);
    }
}