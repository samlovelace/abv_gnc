#include "autobot_grasp_planner/GraspPlannerHandler.h"

#include <limits>
#include <Eigen/Dense>
#include <plog/Log.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "autobot_common/RosTopicManager.h"

namespace {
constexpr const char* kGetTrackedObjectsService = "percep/get_tracked_objects";
constexpr const char* kPlanGraspService = "manip/plan_grasp";
constexpr const char* kGraspPoseTopic = "manip/grasp_pose";

// Below this, a candidate axis is too close to parallel with the fixed
// approach direction (world -z) to give a numerically well-conditioned
// finger-closing direction once projected and normalized.
constexpr double kMinProjectedAxisNorm = 1e-3;

// Projects aRawAxis onto the plane perpendicular to aApproach and normalizes.
// Returns false (axis too close to parallel with aApproach) rather than a
// near-degenerate result.
bool projectPerpendicular(const Eigen::Vector3d& aRawAxis, const Eigen::Vector3d& aApproach, Eigen::Vector3d& aOut)
{
    const Eigen::Vector3d projected = aRawAxis - aRawAxis.dot(aApproach) * aApproach;
    if (projected.norm() < kMinProjectedAxisNorm) return false;
    aOut = projected.normalized();
    return true;
}
}  // namespace

GraspPlannerHandler::GraspPlannerHandler()
{
    auto topicManager = RosTopicManager::getInstance();

    mClientNode = rclcpp::Node::make_shared("grasp_planner_client");
    mGetTrackedObjectsClient =
        mClientNode->create_client<autobot_msgs::srv::GetTrackedObjects>(kGetTrackedObjectsService);

    mPlanGraspService = topicManager->create_service<autobot_msgs::srv::PlanGrasp>(kPlanGraspService,
                            std::bind(&GraspPlannerHandler::planGraspCallback, this,
                                      std::placeholders::_1, std::placeholders::_2));

    topicManager->createPublisher<geometry_msgs::msg::PoseStamped>(kGraspPoseTopic);
}

GraspPlannerHandler::~GraspPlannerHandler() = default;

void GraspPlannerHandler::planGraspCallback(const std::shared_ptr<autobot_msgs::srv::PlanGrasp::Request> aRequest,
                                             std::shared_ptr<autobot_msgs::srv::PlanGrasp::Response> aResponse)
{
    auto request = std::make_shared<autobot_msgs::srv::GetTrackedObjects::Request>();
    request->id = aRequest->id;

    auto future = mGetTrackedObjectsClient->async_send_request(request);
    if (rclcpp::spin_until_future_complete(mClientNode, future, std::chrono::seconds(2)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        aResponse->success = false;
        aResponse->message = "Timed out waiting for " + std::string("percep/get_tracked_objects");
        return;
    }
    auto result = future.get();

    if (result->objects.empty())
    {
        aResponse->success = false;
        aResponse->message = "No tracked object with id=" + std::to_string(aRequest->id);
        return;
    }

    const auto& observation = result->objects[0].observation;

    geometry_msgs::msg::Quaternion orientation;
    if (!computeOrientation(observation, orientation))
    {
        aResponse->success = false;
        aResponse->message = "Object '" + observation.label +
                              "' has no principal axis well-conditioned for a finger-closing direction";
        return;
    }

    double minZ;
    if (!minPointZ(observation.points, minZ))
    {
        aResponse->success = false;
        aResponse->message = "Object '" + observation.label + "' has an empty point cloud";
        return;
    }

    geometry_msgs::msg::PoseStamped graspPose;
    graspPose.header.stamp = RosTopicManager::getInstance()->now();
    graspPose.header.frame_id = observation.points.header.frame_id;
    graspPose.pose.position.x = observation.centroid.x;
    graspPose.pose.position.y = observation.centroid.y;
    graspPose.pose.position.z = minZ;
    graspPose.pose.orientation = orientation;

    RosTopicManager::getInstance()->publishMessage<geometry_msgs::msg::PoseStamped>(kGraspPoseTopic, graspPose);

    aResponse->success = true;
    aResponse->message = "ok";
    aResponse->grasp_pose = graspPose;
}

bool GraspPlannerHandler::computeOrientation(const autobot_msgs::msg::ObjectObservation& aObservation,
                                              geometry_msgs::msg::Quaternion& aQuatOut) const
{
    // Approach direction is always straight down - fixed by the tabletop assumption.
    const Eigen::Vector3d xAxis(0.0, 0.0, -1.0);

    // principal_axes[2] is the smallest-extent ("min thickness") axis - computePca
    // in autobot_obj_segmentation sorts eigenvectors by descending extent, so index
    // 2 is the thinnest. Fall back to principal_axes[1] if [2] is nearly vertical.
    const auto& thinAxisMsg = aObservation.principal_axes[2];
    const Eigen::Vector3d thinAxis(thinAxisMsg.x, thinAxisMsg.y, thinAxisMsg.z);

    Eigen::Vector3d yAxis;
    if (!projectPerpendicular(thinAxis, xAxis, yAxis))
    {
        const auto& midAxisMsg = aObservation.principal_axes[1];
        const Eigen::Vector3d midAxis(midAxisMsg.x, midAxisMsg.y, midAxisMsg.z);
        if (!projectPerpendicular(midAxis, xAxis, yAxis))
        {
            return false;
        }
        LOGW << "Object '" << aObservation.label
             << "': min-thickness axis nearly parallel to approach direction, falling back to middle axis";
    }

    // z completes a right-handed orthonormal frame by construction (x, y are
    // already orthonormal unit vectors).
    const Eigen::Vector3d zAxis = xAxis.cross(yAxis);

    Eigen::Matrix3d rotation;
    rotation.col(0) = xAxis;
    rotation.col(1) = yAxis;
    rotation.col(2) = zAxis;

    const Eigen::Quaterniond q(rotation);
    aQuatOut.x = q.x();
    aQuatOut.y = q.y();
    aQuatOut.z = q.z();
    aQuatOut.w = q.w();
    return true;
}

bool GraspPlannerHandler::minPointZ(const sensor_msgs::msg::PointCloud2& aCloud, double& aMinZOut)
{
    if (aCloud.width * aCloud.height == 0) return false;

    double minZ = std::numeric_limits<double>::max();
    sensor_msgs::PointCloud2ConstIterator<float> iterZ(aCloud, "z");
    for (; iterZ != iterZ.end(); ++iterZ)
    {
        minZ = std::min(minZ, static_cast<double>(*iterZ));
    }

    aMinZOut = minZ;
    return true;
}
