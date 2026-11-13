#include "autobot_manip_executor/ManipExecutorHandler.h"

#include <chrono>

#include <plog/Log.h>

#include "autobot_common/RosTopicManager.h"
#include "autobot_msgs/msg/enable.hpp"
#include "autobot_msgs/msg/joint_position_waypoint.hpp"
#include "autobot_msgs/msg/task_position_waypoint.hpp"

namespace {
constexpr const char* kExecutePickCommandTopic = "manip/execute_pick_command";
constexpr const char* kArmStateTopic = "arm/state";
constexpr const char* kEnableTopic = "arm/enable";
constexpr const char* kJointWaypointTopic = "arm/joint_position_waypoint";
constexpr const char* kTaskWaypointTopic = "arm/task_position_waypoint";
constexpr const char* kGripperWaypointTopic = "arm/gripper_position_waypoint";
constexpr const char* kExecutionStatusTopic = "manip/execution_status";
constexpr const char* kPlanPickService = "manip/plan_pick";
}  // namespace

ManipExecutorHandler::ManipExecutorHandler()
    : mConfig(ManipConfigManager::getInstance()->getConfig()),
      mStateMachine(
          mConfig.executor,
          [](bool aEnabled) {
              autobot_msgs::msg::Enable msg;
              msg.enabled = aEnabled;
              RosTopicManager::getInstance()->publishMessage<autobot_msgs::msg::Enable>(kEnableTopic, msg);
          },
          [](const std::vector<double>& aPositions, const std::vector<double>& aTolerances) {
              autobot_msgs::msg::JointPositionWaypoint msg;
              msg.positions = aPositions;
              msg.tolerances = aTolerances;
              RosTopicManager::getInstance()->publishMessage<autobot_msgs::msg::JointPositionWaypoint>(kJointWaypointTopic, msg);
          },
          [](const geometry_msgs::msg::PoseStamped& aPose, const std::vector<double>& aTolerance) {
              autobot_msgs::msg::TaskPositionWaypoint msg;
              msg.pose = aPose;
              msg.tolerance = aTolerance;
              RosTopicManager::getInstance()->publishMessage<autobot_msgs::msg::TaskPositionWaypoint>(kTaskWaypointTopic, msg);
          },
          [](const std::vector<double>& aPositions) {
              autobot_msgs::msg::JointPositionWaypoint msg;
              msg.positions = aPositions;
              RosTopicManager::getInstance()->publishMessage<autobot_msgs::msg::JointPositionWaypoint>(kGripperWaypointTopic, msg);
          })
{
    auto topicManager = RosTopicManager::getInstance();

    mClientNode = rclcpp::Node::make_shared("manip_executor_client");
    mPlanPickClient = mClientNode->create_client<autobot_msgs::srv::PlanPick>(kPlanPickService);

    topicManager->createSubscriber<autobot_msgs::msg::ExecutePickCommand>(
        kExecutePickCommandTopic, std::bind(&ManipExecutorHandler::executePickCommandCallback, this, std::placeholders::_1));

    topicManager->createSubscriber<autobot_msgs::msg::ManipulatorState>(
        kArmStateTopic, std::bind(&ManipExecutorHandler::armStateCallback, this, std::placeholders::_1));

    topicManager->createPublisher<autobot_msgs::msg::Enable>(kEnableTopic);
    topicManager->createPublisher<autobot_msgs::msg::JointPositionWaypoint>(kJointWaypointTopic);
    topicManager->createPublisher<autobot_msgs::msg::TaskPositionWaypoint>(kTaskWaypointTopic);
    topicManager->createPublisher<autobot_msgs::msg::JointPositionWaypoint>(kGripperWaypointTopic);
    topicManager->createPublisher<autobot_msgs::msg::ExecutionStatus>(kExecutionStatusTopic);

    const auto period =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(1.0 / mConfig.executor.tickRateHz));
    mTickTimer = topicManager->create_wall_timer(period, std::bind(&ManipExecutorHandler::tick, this));
}

ManipExecutorHandler::~ManipExecutorHandler() = default;

void ManipExecutorHandler::executePickCommandCallback(const autobot_msgs::msg::ExecutePickCommand::SharedPtr aMsg)
{
    if (mStateMachine.isRunning())
    {
        LOGW << "Ignoring execute_pick_command for id=" << aMsg->id << " - a pick is already in progress";
        return;
    }

    auto request = std::make_shared<autobot_msgs::srv::PlanPick::Request>();
    request->id = aMsg->id;

    auto future = mPlanPickClient->async_send_request(request);
    if (rclcpp::spin_until_future_complete(mClientNode, future, std::chrono::seconds(5)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        autobot_msgs::msg::ExecutionStatus status;
        status.status = autobot_msgs::msg::ExecutionStatus::FAILED;
        status.message = "Timed out waiting for " + std::string(kPlanPickService);
        RosTopicManager::getInstance()->publishMessage<autobot_msgs::msg::ExecutionStatus>(kExecutionStatusTopic, status);
        return;
    }

    auto result = future.get();
    if (!result->success)
    {
        autobot_msgs::msg::ExecutionStatus status;
        status.status = autobot_msgs::msg::ExecutionStatus::FAILED;
        status.message = "manip/plan_pick failed: " + result->message;
        RosTopicManager::getInstance()->publishMessage<autobot_msgs::msg::ExecutionStatus>(kExecutionStatusTopic, status);
        return;
    }

    mStateMachine.start(result->plan);
    publishStatus();
}

void ManipExecutorHandler::armStateCallback(const autobot_msgs::msg::ManipulatorState::SharedPtr aMsg)
{
    mStateMachine.setLatestArmState(*aMsg);
}

void ManipExecutorHandler::tick()
{
    if (mStateMachine.tick())
    {
        publishStatus();
    }
}

void ManipExecutorHandler::publishStatus()
{
    autobot_msgs::msg::ExecutionStatus status;
    status.waypoint_index = mStateMachine.waypointIndex();
    status.total_waypoints = mStateMachine.totalWaypoints();

    if (mStateMachine.state() == PickExecutorStateMachine::State::SUCCEEDED)
    {
        status.status = autobot_msgs::msg::ExecutionStatus::SUCCEEDED;
        status.message = mStateMachine.message();
    }
    else if (mStateMachine.state() == PickExecutorStateMachine::State::FAILED)
    {
        status.status = autobot_msgs::msg::ExecutionStatus::FAILED;
        status.message = mStateMachine.message();
    }
    else
    {
        status.status = autobot_msgs::msg::ExecutionStatus::IN_PROGRESS;
        status.message = "ok";
    }

    RosTopicManager::getInstance()->publishMessage<autobot_msgs::msg::ExecutionStatus>(kExecutionStatusTopic, status);
}
