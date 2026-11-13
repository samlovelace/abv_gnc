#include "autobot_manip_planner/ManipPlannerHandler.h"

#include <chrono>
#include <optional>

#include <plog/Log.h>
#include <Eigen/Geometry>

#include "autobot_common/RosTopicManager.h"

namespace {
constexpr const char* kPlanGraspService = "manip/plan_grasp";
constexpr const char* kPlanPickService = "manip/plan_pick";
constexpr const char* kHardwareStateTopic = "arm/hardware/state";
constexpr const char* kPickPlanTopic = "manip/pick_plan";
}  // namespace

ManipPlannerHandler::ManipPlannerHandler()
{
    mConfig = ConfigManager::getInstance()->getConfig();

    if (!mKinematics.init(mConfig))
    {
        throw std::runtime_error("ManipPlannerHandler: KinematicsHandler init failed");
    }

    if (!mCollisionModel.init(mConfig.manipulator.urdfPath, mKinematics))
    {
        throw std::runtime_error("ManipPlannerHandler: RobotCollisionModel init failed");
    }

    const KDL::JntArray lower = mKinematics.getJointLimits("lower");
    const KDL::JntArray upper = mKinematics.getJointLimits("upper");
    std::vector<double> lowerVec(lower.data.data(), lower.data.data() + lower.rows());
    std::vector<double> upperVec(upper.data.data(), upper.data.data() + upper.rows());

    mMotionPlanner.init(lowerVec, upperVec, mConfig.planner.planningTimeS, mConfig.planner.stateValidityResolutionFrac,
        [this](const std::vector<double>& aJoints) {
            KDL::JntArray q(aJoints.size());
            for (std::size_t i = 0; i < aJoints.size(); ++i) q(i) = aJoints[i];
            if (!mCollisionModel.updatePoses(q, mKinematics)) return false;
            return !mCollisionChecker.isSelfColliding(mCollisionModel);
        });

    auto topicManager = RosTopicManager::getInstance();

    mClientNode = rclcpp::Node::make_shared("manip_planner_client");
    mPlanGraspClient = mClientNode->create_client<autobot_msgs::srv::PlanGrasp>(kPlanGraspService);

    topicManager->createSubscriber<autobot_msgs::msg::ManipHardwareState>(
        kHardwareStateTopic, std::bind(&ManipPlannerHandler::hardwareStateCallback, this, std::placeholders::_1));

    mPlanPickService = topicManager->create_service<autobot_msgs::srv::PlanPick>(kPlanPickService,
        std::bind(&ManipPlannerHandler::planPickCallback, this, std::placeholders::_1, std::placeholders::_2));

    topicManager->createPublisher<autobot_msgs::msg::PickPlan>(kPickPlanTopic);
}

ManipPlannerHandler::~ManipPlannerHandler() = default;

void ManipPlannerHandler::hardwareStateCallback(const autobot_msgs::msg::ManipHardwareState::SharedPtr aMsg)
{
    std::lock_guard<std::mutex> lk(mStateMutex);
    mLatestState = aMsg;
}

void ManipPlannerHandler::planPickCallback(const std::shared_ptr<autobot_msgs::srv::PlanPick::Request> aRequest,
                                            std::shared_ptr<autobot_msgs::srv::PlanPick::Response> aResponse)
{
    // Deliberate, documented assumption: manip/plan_grasp's grasp_pose
    // (frame "world", from autobot_obj_registry's pipeline) and this node's
    // own KinematicsHandler root frame (also "world" - the synthetic root
    // segment grafted via the manip mount transform) are treated as the same
    // physical origin by configuration convention in this single-robot
    // deployment, not verified by any TF lookup. Revisit if a real TF tree
    // is introduced later.

    auto graspReq = std::make_shared<autobot_msgs::srv::PlanGrasp::Request>();
    graspReq->id = aRequest->id;

    auto future = mPlanGraspClient->async_send_request(graspReq);
    if (rclcpp::spin_until_future_complete(mClientNode, future, std::chrono::seconds(5)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        aResponse->success = false;
        aResponse->message = "Timed out waiting for " + std::string(kPlanGraspService);
        return;
    }

    auto graspResult = future.get();
    if (!graspResult->success)
    {
        aResponse->success = false;
        aResponse->message = "manip/plan_grasp failed: " + graspResult->message;
        return;
    }

    const geometry_msgs::msg::PoseStamped graspPose = graspResult->grasp_pose;
    const geometry_msgs::msg::PoseStamped preGraspPose = computePreGraspPose(graspPose, mConfig.planner.approachDistanceM);

    autobot_msgs::msg::ManipHardwareState::SharedPtr stateCopy;
    {
        std::lock_guard<std::mutex> lk(mStateMutex);
        stateCopy = mLatestState;
    }
    if (!stateCopy)
    {
        aResponse->success = false;
        aResponse->message = "No arm/hardware/state received yet";
        return;
    }

    const int nrJoints = mKinematics.getNrJoints();
    if (static_cast<int>(stateCopy->position.size()) < nrJoints)
    {
        aResponse->success = false;
        aResponse->message = "arm/hardware/state has fewer joints than expected";
        return;
    }

    KDL::JntArray currentQ(nrJoints);
    for (int i = 0; i < nrJoints; ++i)
    {
        currentQ(i) = stateCopy->position[i];
    }

    const KDL::Frame goalFrame = poseToFrame(preGraspPose.pose);
    KDL::JntArray ikSolution(nrJoints);
    if (!mKinematics.solveIK(currentQ, goalFrame, ikSolution))
    {
        aResponse->success = false;
        aResponse->message = "IK failed to converge at pre-grasp pose";
        return;
    }

    if (!mCollisionModel.updatePoses(ikSolution, mKinematics))
    {
        aResponse->success = false;
        aResponse->message = "Failed to compute collision geometry for the IK solution";
        return;
    }

    std::string why;
    if (mCollisionChecker.isSelfColliding(mCollisionModel, &why))
    {
        aResponse->success = false;
        aResponse->message = "No collision-free IK solution at pre-grasp pose: " + why;
        return;
    }

    std::vector<KDL::JntArray> jointPath;
    std::string plannerMsg;
    if (!mMotionPlanner.plan(currentQ, ikSolution, jointPath, plannerMsg))
    {
        aResponse->success = false;
        aResponse->message = "Motion planning failed: " + plannerMsg;
        return;
    }

    const autobot_msgs::msg::PickPlan plan = assemblePlan(jointPath, preGraspPose, graspPose);
    RosTopicManager::getInstance()->publishMessage<autobot_msgs::msg::PickPlan>(kPickPlanTopic, plan);

    aResponse->success = true;
    aResponse->message = "ok";
    aResponse->plan = plan;
}

geometry_msgs::msg::PoseStamped ManipPlannerHandler::computePreGraspPose(
    const geometry_msgs::msg::PoseStamped& aGraspPose, double aApproachDistanceM)
{
    const Eigen::Quaterniond q(aGraspPose.pose.orientation.w, aGraspPose.pose.orientation.x,
                                aGraspPose.pose.orientation.y, aGraspPose.pose.orientation.z);
    const Eigen::Vector3d approachAxis = q.toRotationMatrix().col(0);  // local +x, in world frame

    geometry_msgs::msg::PoseStamped out = aGraspPose;
    out.pose.position.x -= aApproachDistanceM * approachAxis.x();
    out.pose.position.y -= aApproachDistanceM * approachAxis.y();
    out.pose.position.z -= aApproachDistanceM * approachAxis.z();
    return out;
}

KDL::Frame ManipPlannerHandler::poseToFrame(const geometry_msgs::msg::Pose& aPose)
{
    const KDL::Rotation r = KDL::Rotation::Quaternion(aPose.orientation.x, aPose.orientation.y,
                                                        aPose.orientation.z, aPose.orientation.w);
    const KDL::Vector p(aPose.position.x, aPose.position.y, aPose.position.z);
    return KDL::Frame(r, p);
}

autobot_msgs::msg::PickPlan ManipPlannerHandler::assemblePlan(const std::vector<KDL::JntArray>& aJointPath,
                                                                const geometry_msgs::msg::PoseStamped& aPreGraspPose,
                                                                const geometry_msgs::msg::PoseStamped& aGraspPose) const
{
    autobot_msgs::msg::PickPlan plan;
    plan.header.stamp = RosTopicManager::getInstance()->now();
    plan.header.frame_id = mKinematics.getRootFrame();

    for (const auto& q : aJointPath)
    {
        autobot_msgs::msg::PickWaypoint wp;
        wp.type = autobot_msgs::msg::PickWaypoint::JOINT_TRAJECTORY;
        wp.joint_positions.assign(q.data.data(), q.data.data() + q.rows());
        plan.waypoints.push_back(std::move(wp));
    }

    const std::optional<std::vector<double>> openPos = mConfig.gripper ? mConfig.gripper->openPos : std::nullopt;
    const std::optional<std::vector<double>> closedPos = mConfig.gripper ? mConfig.gripper->closedPos : std::nullopt;

    auto makeTaskWp = [](uint8_t aType, const geometry_msgs::msg::PoseStamped& aPose,
                          const std::optional<std::vector<double>>& aGripperPos) {
        autobot_msgs::msg::PickWaypoint wp;
        wp.type = aType;
        wp.task_pose = aPose;
        if (aGripperPos) wp.gripper_positions = *aGripperPos;
        return wp;
    };

    plan.waypoints.push_back(makeTaskWp(autobot_msgs::msg::PickWaypoint::PRE_GRASP, aPreGraspPose, openPos));
    plan.waypoints.push_back(makeTaskWp(autobot_msgs::msg::PickWaypoint::GRASP, aGraspPose, openPos));
    plan.waypoints.push_back(makeTaskWp(autobot_msgs::msg::PickWaypoint::TOGGLE, aGraspPose, closedPos));
    plan.waypoints.push_back(makeTaskWp(autobot_msgs::msg::PickWaypoint::LIFT, aPreGraspPose, closedPos));

    return plan;
}
