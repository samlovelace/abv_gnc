#ifndef CONFIGURATIONS_H
#define CONFIGURATIONS_H

#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <plog/Log.h>

struct VehicleConfig 
{
    std::string Name;
    double Mass;
    double Inertia;
};

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

struct HeartbeatConfig
{
    double mRate;         // Hz - how often each node publishes its heartbeat
    double mStaleAfter;   // seconds - GUI-side threshold before a node is considered disconnected
    int mPingIntervalMs;  // GUI-side: how often to ping the robot host for the Comms indicator
};

struct TableViewConfig
{
    double mWidth;       // meters - table X extent, GUI-side top-down view
    double mHeight;      // meters - table Y extent
    double mRobotWidth;  // meters - robot footprint used to draw the glyph
    double mRobotLength; // meters
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

struct ObjDetectionConfig
{
    std::string mType;        // which IDetector implementation, e.g. "yolo11-seg"
    std::string mModelPath;
    std::vector<std::string> mClassNames;  // index == class channel in the model output

    int mInputSize;
    double mConfThresh;
    double mIouThresh;
    bool mUseCuda;
};

struct ObjSegmentationConfig
{
    double mSyncSlopSec;      // ApproximateTimeSynchronizer max interval duration
    int mMinPointsPerObject;  // skip + warn if a detection's valid point count is below this
    int mMaxPointsPerObject;  // 0 = unbounded; else caps PointCloud2 size / PCA cost for huge masks

    int mMaskErosionKernelSize;      // pixels, elliptical kernel; <=1 disables erosion
    int mOutlierMeanK;                // neighbors per point for outlier stats; <=0 disables
    double mOutlierStdDevMultiplier;  // reject if meanNeighborDist > mean + this*stddev; <=0 disables
};

struct ObjRegistryConfig
{
    double mMatchDistanceThresholdM;  // meters; max centroid distance to same-label existing track to count as a match
    double mStalenessTimeoutSec;      // seconds since last_seen before a tracked object is pruned (erased)
    double mPruneRateHz;              // rate of the prune/staleness-check wall timer
};

namespace ConfigUtils
{
    static Eigen::Vector3d parseVector3d(const YAML::Node& node) 
    {
        Eigen::Vector3d vec;

        if (node && node.IsSequence() && node.size() == 3) {
            vec[0] = node[0].as<double>();
            vec[1] = node[1].as<double>();
            vec[2] = node[2].as<double>();
        } 
        else 
        {
            throw std::runtime_error("Invalid Vector3d format in YAML.");
        }

        return vec;
    }

    static std::vector<int> parseIntVector(const YAML::Node& node)
    {
        if (!node || !node.IsSequence()) 
        {
            throw std::runtime_error("Invalid int vector format in YAML.");
        }

        std::vector<int> vec;
        vec.reserve(node.size());  // Optional, improves efficiency

        for (const auto& element : node) 
        {
            vec.push_back(element.as<int>());
        }

        return vec;
    }
}

#endif // CONFIGURATIONS_H