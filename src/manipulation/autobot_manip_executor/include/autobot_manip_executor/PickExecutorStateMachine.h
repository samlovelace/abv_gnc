#ifndef PICK_EXECUTOR_STATE_MACHINE_H
#define PICK_EXECUTOR_STATE_MACHINE_H

#include <chrono>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "autobot_config/ManipConfigManager.h"
#include "autobot_msgs/msg/pick_plan.hpp"
#include "autobot_msgs/msg/pick_waypoint.hpp"
#include "autobot_msgs/msg/manipulator_state.hpp"

// Dispatches a PickPlan to autobot_manip_controller one waypoint at a time,
// waiting for arrival (or, for the gripper, a fixed dwell - no arrival
// signal exists for it) before moving to the next. Does no publishing/
// subscribing itself - the owning handler injects dispatch callbacks and
// feeds it the latest measured arm state; this class only holds the
// sequencing logic.
class PickExecutorStateMachine
{
public:
    enum class State
    {
        IDLE,
        ENABLING,
        DISPATCH_WAYPOINT,
        WAITING_ARM_ARRIVAL,
        DISPATCH_GRIPPER,
        WAITING_GRIPPER_DWELL,
        SUCCEEDED,
        FAILED
    };

    using EnableFn = std::function<void(bool)>;
    using JointWaypointFn = std::function<void(const std::vector<double>& aPositions, const std::vector<double>& aTolerances)>;
    using TaskWaypointFn = std::function<void(const geometry_msgs::msg::PoseStamped& aPose, const std::vector<double>& aTolerance)>;
    using GripperWaypointFn = std::function<void(const std::vector<double>& aPositions)>;

    PickExecutorStateMachine(const ManipConfigManager::ExecutorConfig& aConfig,
                              EnableFn aEnableFn,
                              JointWaypointFn aJointWaypointFn,
                              TaskWaypointFn aTaskWaypointFn,
                              GripperWaypointFn aGripperWaypointFn);

    // Resets the waypoint cursor and begins executing aPlan. Call only when
    // !isRunning() - the handler is responsible for rejecting a new command
    // while one is already in progress.
    void start(const autobot_msgs::msg::PickPlan& aPlan);

    // Thread-safe: called from the arm/state subscription callback.
    void setLatestArmState(const autobot_msgs::msg::ManipulatorState& aState);

    // Advances the state machine by one step. Returns true if the state
    // changed this call (so the handler knows to publish a status update).
    bool tick();

    State state() const { return mState; }
    bool isRunning() const;
    bool succeeded() const { return mState == State::SUCCEEDED; }
    int waypointIndex() const { return mWaypointIndex; }
    int totalWaypoints() const { return static_cast<int>(mPlan.waypoints.size()); }
    const std::string& message() const { return mMessage; }

private:
    void setState(State aState);

    // True if wp is a JOINT_TRAJECTORY waypoint and every measured joint
    // position is within (jointTolerance * inflatedToleranceFactor) of the
    // waypoint's target. Used only as a timeout fallback, never as the
    // primary arrival check. Caller must already hold mArmStateMutex.
    bool withinInflatedTolerance(const autobot_msgs::msg::PickWaypoint& wp) const;

    ManipConfigManager::ExecutorConfig mConfig;
    EnableFn mEnableFn;
    JointWaypointFn mJointWaypointFn;
    TaskWaypointFn mTaskWaypointFn;
    GripperWaypointFn mGripperWaypointFn;

    State mState{State::IDLE};
    autobot_msgs::msg::PickPlan mPlan;
    int mWaypointIndex{0};
    std::chrono::steady_clock::time_point mDispatchTime;
    std::string mMessage;

    std::mutex mArmStateMutex;
    autobot_msgs::msg::ManipulatorState mLatestArmState;
    bool mHasArmState{false};
};
#endif  // PICK_EXECUTOR_STATE_MACHINE_H
