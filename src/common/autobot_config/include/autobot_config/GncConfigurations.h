#pragma once

#include <array>
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>

struct GuidanceConfig
{
    int mStateMachineRate;

    // Default max time (seconds) to wait for arrival at a single waypoint
    // before falling back to the relaxed-tolerance check. Path generators
    // may override this per-waypoint (see Waypoint::mTimeout).
    double mWaypointTimeout;

    // Multiplier applied to the commanded arrival tolerance when a waypoint
    // timeout fires, to decide whether the current pose is "close enough"
    // to move on rather than getting stuck.
    double mWaypointTimeoutToleranceScale;
};

struct NavigationConfig
{
    std::string mInterface;
    int mRate;
    std::string mServerIp;
    std::string mLocalIp;
    std::string mRigidBodyName;

    // Max duration (seconds) the EKF may free-run (predict-only, no real
    // measurement) before its output is frozen and marked invalid.
    double mMaxDeadReckonDuration;

    // autobot_simulator only: enables its synthetic burst-dropout mechanism
    // (used to exercise disconnection handling end-to-end). Off by default.
    bool mSimulateDropout;
};

struct SceneConfig
{
    // meters - the 2D region (world frame, same convention as Vec3/State)
    // the ABV must remain within. Consumed by autobot_guidance's Scene for
    // table-bounds containment checks.
    double mXMin;
    double mXMax;
    double mYMin;
    double mYMax;
};

struct CollisionAvoidConfig
{
    double mGridResolution; // meters/cell for CollisionAvoidPathGenerator's A* search grid
    double mReplanRate;     // Hz - background re-solve rate
};

struct ControlConfig
{
    int mStateMachineRate;

    // Controller arrival
    Eigen::Vector3d mPoseArrivalTol;
    double mArrivalDuration;

    // Max time (seconds) since the last abv/state message was received
    // before nav data is considered stale (catches full topic/node dropout).
    double mNavDataTimeout;

    double mForce;
    double mMomentArm;
    // Vehicle dynamics (used by the simulator)
    double mMass;
    double mInertia;

    // true iff Dynamics.PropagationMode is "external": autobot_simulator publishes
    // a net body wrench and sources state (for damping + the nav/Optitrack
    // publish path) from Gazebo instead of integrating locally.
    bool mUseExternalPropagation;
    std::string mExternalStateTopic;  // autobot_msgs/State, from autobot_bridge's GazeboStateConvertor
    std::string mWrenchCommandTopic;  // geometry_msgs/WrenchStamped, to ptera_sim's WrenchApplicator
    std::string mWrenchTargetLink;    // stamped as the outgoing wrench's frame_id
};

struct ControlLawConfig
{
    // which IControlLaw implementation autobot_control should instantiate.
    // "PID" today; room for "LQR"/"MPC" etc. as new IControlLaw implementations
    // are added, without needing a new node.
    std::string mLawType;

    // PID controller gains
    Eigen::Vector3d mKp;
    Eigen::Vector3d mKi;
    Eigen::Vector3d mKd;
};

struct ThrusterConfig
{
    // ThrusterCommander & ThrusterDriver configs
    std::string mThrusterDriverType;
    std::vector<int> mGpioPins;
    Eigen::Vector3d mSchmittTriggerOn;
    Eigen::Vector3d mSchmittTriggerOff;

    // which IThrusterMapper implementation ThrusterCommander uses: "LookupTable" (default, current behavior) | "Matrix"
    std::string mThrusterAllocationStrategy;

    // Control allocation matrix rows: contribution of each of the 8 thrusters
    // to (fx, fy, tz) when that thruster fires alone (unitless; mForce/mMomentArm
    // scale these afterward). Matrix strategy only.
    std::vector<int> mAllocationX;    // size 8
    std::vector<int> mAllocationY;    // size 8
    std::vector<int> mAllocationYaw;  // size 8
    double mAllocationThreshold;      // firing cutoff on the pseudo-inverse solution. Matrix strategy only.

    double mForce;
    double mMomentArm;
};
