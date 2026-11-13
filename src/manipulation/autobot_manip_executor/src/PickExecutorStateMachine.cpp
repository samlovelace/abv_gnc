#include "autobot_manip_executor/PickExecutorStateMachine.h"

#include <plog/Log.h>

#include "autobot_msgs/msg/pick_waypoint.hpp"

PickExecutorStateMachine::PickExecutorStateMachine(const ConfigManager::ExecutorConfig& aConfig,
                                                     EnableFn aEnableFn,
                                                     JointWaypointFn aJointWaypointFn,
                                                     TaskWaypointFn aTaskWaypointFn,
                                                     GripperWaypointFn aGripperWaypointFn)
    : mConfig(aConfig), mEnableFn(std::move(aEnableFn)), mJointWaypointFn(std::move(aJointWaypointFn)),
      mTaskWaypointFn(std::move(aTaskWaypointFn)), mGripperWaypointFn(std::move(aGripperWaypointFn))
{
}

bool PickExecutorStateMachine::isRunning() const
{
    return mState != State::IDLE && mState != State::SUCCEEDED && mState != State::FAILED;
}

void PickExecutorStateMachine::start(const autobot_msgs::msg::PickPlan& aPlan)
{
    mPlan = aPlan;
    mWaypointIndex = 0;
    mMessage.clear();
    setState(State::ENABLING);
}

void PickExecutorStateMachine::setLatestArmState(const autobot_msgs::msg::ManipulatorState& aState)
{
    std::lock_guard<std::mutex> lk(mArmStateMutex);
    mLatestArmState = aState;
    mHasArmState = true;
}

void PickExecutorStateMachine::setState(State aState)
{
    mState = aState;
}

bool PickExecutorStateMachine::tick()
{
    const State before = mState;

    switch (mState)
    {
        case State::IDLE:
        case State::SUCCEEDED:
        case State::FAILED:
            break;

        case State::ENABLING:
            mEnableFn(true);
            setState(State::DISPATCH_WAYPOINT);
            break;

        case State::DISPATCH_WAYPOINT:
        {
            if (mWaypointIndex >= totalWaypoints())
            {
                mMessage = "ok";
                setState(State::SUCCEEDED);
                break;
            }

            const auto& wp = mPlan.waypoints[mWaypointIndex];
            if (wp.type == autobot_msgs::msg::PickWaypoint::JOINT_TRAJECTORY)
            {
                const std::vector<double> tolerances(wp.joint_positions.size(), mConfig.jointTolerance);
                mJointWaypointFn(wp.joint_positions, tolerances);
            }
            else
            {
                const std::vector<double> tolerance = {
                    mConfig.taskPositionToleranceM, mConfig.taskPositionToleranceM, mConfig.taskPositionToleranceM,
                    mConfig.taskOrientationToleranceRad, mConfig.taskOrientationToleranceRad, mConfig.taskOrientationToleranceRad};
                mTaskWaypointFn(wp.task_pose, tolerance);
            }

            mDispatchTime = std::chrono::steady_clock::now();
            setState(State::WAITING_ARM_ARRIVAL);
            break;
        }

        case State::WAITING_ARM_ARRIVAL:
        {
            bool arrived = false;
            {
                std::lock_guard<std::mutex> lk(mArmStateMutex);
                arrived = mHasArmState && mLatestArmState.state == autobot_msgs::msg::ManipulatorState::ARRIVED;
            }

            if (arrived)
            {
                const auto& wp = mPlan.waypoints[mWaypointIndex];
                if (!wp.gripper_positions.empty())
                {
                    setState(State::DISPATCH_GRIPPER);
                }
                else
                {
                    ++mWaypointIndex;
                    setState(State::DISPATCH_WAYPOINT);
                }
            }
            else
            {
                const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - mDispatchTime).count();
                if (elapsed > mConfig.armArrivalTimeoutSec)
                {
                    mMessage = "Timed out waiting for arm arrival at waypoint " + std::to_string(mWaypointIndex);
                    LOGW << mMessage;
                    setState(State::FAILED);
                }
            }
            break;
        }

        case State::DISPATCH_GRIPPER:
        {
            const auto& wp = mPlan.waypoints[mWaypointIndex];
            mGripperWaypointFn(wp.gripper_positions);
            mDispatchTime = std::chrono::steady_clock::now();
            setState(State::WAITING_GRIPPER_DWELL);
            break;
        }

        case State::WAITING_GRIPPER_DWELL:
        {
            const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - mDispatchTime).count();
            if (elapsed > mConfig.gripperDwellSec)
            {
                ++mWaypointIndex;
                setState(State::DISPATCH_WAYPOINT);
            }
            break;
        }
    }

    return mState != before;
}
