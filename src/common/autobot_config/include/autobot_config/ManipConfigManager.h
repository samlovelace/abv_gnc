#pragma once

#include <string>
#include <vector>
#include <optional>
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <kdl/frames.hpp>

#include "autobot_config/YamlUtil.h"

// ---------------------------------------------------------------------------
// ManipConfigManager — singleton that loads autobot_manip_bringup config and manip.yaml.
//
// Usage:
//   ManipConfigManager::getInstance()->loadConfig();
//   const auto& cfg = ManipConfigManager::getInstance()->getConfig();
//
// Path resolution (all derived from the autobot_manip_bringup package share dir):
//   config.yaml   →  <share>/config/config.yaml
//   manip.yaml    →  <share>/config/manipulators/<Manipulator.name>/manip.yaml
//   base.yaml     →  <autobot_bringup share>/config/mobile_base/<MobileBase.name>/base.yaml (optional)
// ---------------------------------------------------------------------------
class ManipConfigManager
{
public:
    // -----------------------------------------------------------------------
    // Sub-structs — mirror the YAML hierarchy; used for per-subsystem access
    // -----------------------------------------------------------------------

    struct MobileBaseConfig
    {
        std::string              urdfPath;        // resolved absolute path
        std::string              manipAttachLink; // link on the base where the arm attaches
        std::vector<std::string> jointNames;
        std::vector<double>      initialPositions;
        std::vector<double>      accelLimits;
        std::vector<double>      jerkLimits;

        // Parallel to jointNames: which rigid-body DOF of the base pose each
        // virtual joint represents — one of x, y, z, roll, pitch, yaw
        std::vector<std::string> jointAxes;

        // false: base joints stay in the kinematics (measured, held fixed by
        // IK) but are excluded from trajectory generation and commands
        bool        controllable = false;
        std::string commandTopic;    // VehicleWaypoint topic when controllable
    };

    struct ManipulatorConfig
    {
        std::string              urdfPath;   // resolved absolute path
        std::string              baseLink;
        std::string              endLink;
        std::vector<std::string> jointNames;
        std::vector<double>      initialPositions;
        std::vector<double>      accelLimits;
        std::vector<double>      jerkLimits;

        // Named joint-space presets (optional, arm joints only)
        std::optional<std::vector<double>> deployPos;
        std::optional<std::vector<double>> stowPos;
    };

    // Gripper joints — deliberately kept separate from ManipulatorConfig and
    // never merged into Config's flat jointNames/limits vectors. These joints
    // must never enter FK/IK or WaypointExecutor's trajectory generation.
    struct GripperConfig
    {
        std::vector<std::string> jointNames;
        std::vector<double>      initialPositions;

        // Named joint-space presets (optional)
        std::optional<std::vector<double>> openPos;
        std::optional<std::vector<double>> closedPos;
    };

    struct PlannerConfig
    {
        double approachDistanceM = 0.05;            // pre-grasp backoff distance, meters
        double planningTimeS = 2.0;                 // OMPL time budget, seconds
        double stateValidityResolutionFrac = 0.01;  // OMPL setLongestValidSegmentFraction
    };

    struct ExecutorConfig
    {
        double armArrivalTimeoutSec = 30.0;         // per-waypoint timeout waiting for arm/state == ARRIVED
        double gripperDwellSec = 1.0;               // fixed settle time after a gripper command (no arrival signal exists)
        double tickRateHz = 20.0;                   // state-machine tick rate
        double jointTolerance = 0.01;                // rad, applied uniformly to every dispatched joint waypoint
        double inflatedToleranceFactor = 3.0;        // multiplier on jointTolerance: on a JOINT_TRAJECTORY
                                                      // waypoint arrival timeout, accept if every joint is within
                                                      // jointTolerance * this instead of failing outright
        double taskPositionToleranceM = 0.01;        // m, xyz
        double taskOrientationToleranceRad = 0.05;   // rad, rpy
    };

    struct Config
    {
        // Top-level
        std::string manipName;      // config.yaml Manipulator.name, e.g. "abv_rebot"
        std::string manipType;      // e.g. "dynamixel"
        std::string commsMode;      // "simulated" | "hardware"
        int         controlRateHz = 10;

        // Per-subsystem configs — use when addressing the mobile base or arm
        // joints independently (e.g. IK solvers, named presets).
        // mobileBase is nullopt for a standalone manipulator (not mounted on
        // anything selectable — e.g. a fixed-base arm).
        std::optional<MobileBaseConfig>  mobileBase;
        ManipulatorConfig                manipulator;
        std::optional<GripperConfig>     gripper;
        PlannerConfig                    planner;  // defaults apply if config.yaml's Planner: block is absent
        ExecutorConfig                   executor; // defaults apply if config.yaml's Executor: block is absent

        // Combined flat vectors — ordered [mobile base joints..., arm joints...]
        // (or arm joints only, if there's no mobile base).
        // Use for whole-body planners, limit arrays, joint state indexing, etc.
        std::vector<std::string> jointNames;
        std::vector<double>      initialPositions;
        std::vector<double>      accelLimits;
        std::vector<double>      jerkLimits;

        // Controlled subset of the combined vectors — the combined vectors
        // from ctrlOffset onward. ctrlOffset is the mobile base joint count
        // when the base isn't controllable, else 0. Use for trajectory
        // generation, joint-space waypoints and hardware commands.
        int                      ctrlOffset = 0;
        std::vector<std::string> ctrlJointNames;
        std::vector<double>      ctrlInitialPositions;
        std::vector<double>      ctrlAccelLimits;
        std::vector<double>      ctrlJerkLimits;

        // Transform: attach point (mobile base's attach link, or world if
        // standalone) -> manipulator base frame
        KDL::Frame T_V_B;
    };

    // -----------------------------------------------------------------------
    // Singleton access
    // -----------------------------------------------------------------------
    static ManipConfigManager* getInstance()
    {
        static ManipConfigManager instance;
        return &instance;
    }

    // -----------------------------------------------------------------------
    // Resolve paths from autobot_manip_bringup share dir, load config.yaml and the
    // manip.yaml it references. Throws std::runtime_error on any problem.
    // -----------------------------------------------------------------------
    void loadConfig();

    const Config&     getConfig()  const { return mConfig; }
    const YAML::Node& getRawNode() const { return mManipRoot; }  // manip.yaml root

    // Generic nested accessor into manip.yaml: getValue<T>("manipulator.base_link")
    template<typename T>
    T getValue(const std::string& dotPath) const
    {
        YAML::Node current = YAML::Clone(mManipRoot);
        for (const auto& key : autobot_config::YamlUtil::splitPath(dotPath))
        {
            if (!current[key])
                throw std::runtime_error("[ManipConfigManager] YAML path not found: " + dotPath);
            current = current[key];
        }
        return current.as<T>();
    }

private:
    ManipConfigManager() = default;
    ~ManipConfigManager() = default;
    ManipConfigManager(const ManipConfigManager&) = delete;
    ManipConfigManager& operator=(const ManipConfigManager&) = delete;

    // -----------------------------------------------------------------------
    // Section loaders
    // -----------------------------------------------------------------------
    void loadMobileBaseSection (const std::filesystem::path& baseDir, const YAML::Node& aBaseSelectNode);
    void loadManipulatorSection();
    void loadGripperSection    ();
    void loadMountSection      ();
    void loadPlannerSection    (const YAML::Node& aTopConfig);
    void loadExecutorSection   (const YAML::Node& aTopConfig);
    void mergeJointData        ();

    // -----------------------------------------------------------------------
    // State
    // -----------------------------------------------------------------------
    Config     mConfig;
    YAML::Node mManipRoot;  // root of the active manip.yaml
    YAML::Node mBaseRoot;   // root of the active mobile_base's base.yaml (if any)
};
