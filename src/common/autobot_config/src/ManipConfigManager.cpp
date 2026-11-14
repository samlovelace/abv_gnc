#include "autobot_config/ManipConfigManager.h"
#include "autobot_config/Util.hpp"
#include "plog/Log.h"
#include <stdexcept>
#include <filesystem>
#include <set>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace fs = std::filesystem;
using namespace autobot_config::YamlUtil;

void ManipConfigManager::loadConfig()
{
    // Load top-level config.yaml to find which manipulator to use
    const fs::path configPath = fs::path(util::getConfigPath("manipulation.yaml")); 
    if (!fs::exists(configPath))
    {    
        throw std::runtime_error("[ManipConfigManager] Top-level config not found: " + configPath.string());
    }

    LOGI << "Loading config: " << configPath.string();
    const YAML::Node topConfig = YAML::LoadFile(configPath.string());

    const YAML::Node manipNode = require(topConfig, "Manipulator", "manipulation.yaml");
    const std::string manipName = requireScalar<std::string>(manipNode, "name", "Manipulator");

    // Resolve manipulator.yaml from the manipulator description package
    const fs::path shareDir = ament_index_cpp::get_package_share_directory(manipName + "_description"); 
    const fs::path manipYamlPath = shareDir / "config" / "manipulator.yaml";
    if (!fs::exists(manipYamlPath))
    {
        throw std::runtime_error(
            "[ManipConfigManager] manipulator.yaml not found for manipulator '" + manipName +
            "': " + manipYamlPath.string());
    }

    LOGI << "Loading manipulator config: " << manipYamlPath.string();
    mManipRoot = YAML::LoadFile(manipYamlPath.string());

    mConfig.manipName = manipName;

    // Top-level manipulator.yaml fields
    mConfig.manipType     = requireScalar<std::string>(mManipRoot, "type",  "manipulator.yaml");
    mConfig.commsMode     = requireScalar<std::string>(mManipRoot, "comms", "manipulator.yaml");
    mConfig.controlRateHz = optionalScalar<int>(mManipRoot, "rate", 10);

    loadManipulatorSection();
    loadGripperSection    ();
    loadMountSection      ();
    loadPlannerSection    (topConfig);
    loadExecutorSection   (topConfig);

    // Mobile base is optional — a manipulator with no MobileBase entry is standalone
    if (topConfig["MobileBase"] && !topConfig["MobileBase"].IsNull())
    {
        const std::string baseName = requireScalar<std::string>(topConfig["MobileBase"], "name", "MobileBase");

        const fs::path bringupShareDir = []{
            try {
                return fs::path(ament_index_cpp::get_package_share_directory("autobot_bringup"));
            } catch (const std::exception& e) {
                throw std::runtime_error(
                    std::string("[ManipConfigManager] Could not locate autobot_bringup package: ") + e.what());
            }
        }();

        const fs::path baseDir     = bringupShareDir / "config" / "mobile_base" / baseName;
        const fs::path baseYamlPath = baseDir / "base.yaml";
        if (!fs::exists(baseYamlPath))
            throw std::runtime_error(
                "[ManipConfigManager] base.yaml not found for mobile base '" + baseName +
                "': " + baseYamlPath.string());

        LOGI << "Loading mobile base config: " << baseYamlPath.string();
        mBaseRoot = YAML::LoadFile(baseYamlPath.string());

        loadMobileBaseSection(baseDir, topConfig["MobileBase"]);
    }

    mergeJointData();

    LOGI << "Config loaded successfully — manipulator: '" << manipName
         << "', " << mConfig.jointNames.size() << " total joints ("
         << (mConfig.mobileBase ? mConfig.mobileBase->jointNames.size() : 0) << " mobile base, "
         << mConfig.manipulator.jointNames.size()  << " arm), "
         << mConfig.ctrlJointNames.size() << " controlled";
    LOGD << YAML::Dump(mManipRoot);
}

// ---------------------------------------------------------------------------
// mobile_base/<name>/base.yaml:
// ---------------------------------------------------------------------------
void ManipConfigManager::loadMobileBaseSection(const fs::path& baseDir, const YAML::Node& aBaseSelectNode)
{
    const std::string ctx = "mobile_base";

    MobileBaseConfig mb;
    mb.urdfPath        = resolvePath(baseDir, requireScalar<std::string>(mBaseRoot, "urdf", ctx));
    mb.manipAttachLink = requireScalar<std::string>(mBaseRoot, "manip_attach_link", ctx);

    auto jointNames = requireSeq<std::string>(mBaseRoot, "joint_names", ctx);
    const std::size_t n = jointNames.size();
    mb.jointNames = std::move(jointNames);

    mb.initialPositions = requireSeq<double>(mBaseRoot, "initial_positions", ctx, n);
    mb.accelLimits      = requireSeq<double>(mBaseRoot, "accel_limits",      ctx, n);
    mb.jerkLimits       = requireSeq<double>(mBaseRoot, "jerk_limits",       ctx, n);
    mb.jointAxes        = requireSeq<std::string>(mBaseRoot, "joint_axes",   ctx, n);

    static const std::set<std::string> validAxes = {"x", "y", "z", "roll", "pitch", "yaw"};
    std::set<std::string> seenAxes;
    for (const auto& axis : mb.jointAxes)
    {
        if (!validAxes.count(axis))
            throw std::runtime_error(
                "[ManipConfigManager] Unknown axis '" + axis + "' in '" + ctx +
                "' joint_axes, expected one of x, y, z, roll, pitch, yaw");
        if (!seenAxes.insert(axis).second)
            throw std::runtime_error(
                "[ManipConfigManager] Duplicate axis '" + axis + "' in '" + ctx + "' joint_axes");
    }

    // controllable is a per-deployment choice, so it lives in config.yaml's MobileBase entry
    mb.controllable = optionalScalar<bool>(aBaseSelectNode, "controllable", false);
    mb.commandTopic = optionalScalar<std::string>(mBaseRoot, "command_topic", "robot/vehicle/waypoint");

    mConfig.mobileBase = std::move(mb);
}

void ManipConfigManager::loadManipulatorSection()
{
    const std::string ctx = "manipulator.yaml";
    const YAML::Node& m = mManipRoot;

    const std::string urdfPackage = requireScalar<std::string>(m, "urdf_package", ctx);
    const std::string urdfPath    = requireScalar<std::string>(m, "urdf_path", ctx);
    const fs::path urdfPackageShareDir = []( const std::string& aPackage ){
        try {
            return fs::path(ament_index_cpp::get_package_share_directory(aPackage));
        } catch (const std::exception& e) {
            throw std::runtime_error(
                std::string("[ManipConfigManager] Could not locate '") + aPackage + "' package: " + e.what());
        }
    }(urdfPackage);
    mConfig.manipulator.urdfPath = resolvePath(urdfPackageShareDir, urdfPath);
    mConfig.manipulator.baseLink = requireScalar<std::string>(m, "base_link", ctx);
    mConfig.manipulator.endLink  = requireScalar<std::string>(m, "end_link",  ctx);

    auto jointNames = requireSeq<std::string>(m, "joint_names", ctx);
    const std::size_t n = jointNames.size();
    mConfig.manipulator.jointNames = std::move(jointNames);

    mConfig.manipulator.initialPositions = requireSeq<double>(m, "initial_positions", ctx, n);
    mConfig.manipulator.accelLimits      = requireSeq<double>(m, "accel_limits",      ctx, n);
    mConfig.manipulator.jerkLimits       = requireSeq<double>(m, "jerk_limits",       ctx, n);

    // Named presets validated against arm joint count only
    mConfig.manipulator.deployPos = optionalSeq<double>(m["deploy"], "pos", n);
    mConfig.manipulator.stowPos   = optionalSeq<double>(m["stow"],   "pos", n);
}

// ---------------------------------------------------------------------------
// manipulator.yaml: gripper (optional — absent for a manipulator with no gripper).
// Deliberately not merged into Config's flat joint vectors — those feed
// WaypointExecutor/KinematicsHandler and must stay arm(+base)-only.
// ---------------------------------------------------------------------------
void ManipConfigManager::loadGripperSection()
{
    const YAML::Node& m = mManipRoot;

    if (!m["gripper"] || m["gripper"].IsNull())
        return;

    const std::string ctx = "gripper";
    const YAML::Node& g = m["gripper"];

    GripperConfig gripper;

    auto jointNames = requireSeq<std::string>(g, "joint_names", ctx);
    const std::size_t n = jointNames.size();
    gripper.jointNames = std::move(jointNames);

    gripper.initialPositions = requireSeq<double>(g, "initial_positions", ctx, n);

    gripper.openPos   = optionalSeq<double>(g["open"],   "pos", n);
    gripper.closedPos = optionalSeq<double>(g["closed"], "pos", n);

    mConfig.gripper = std::move(gripper);
}

// ---------------------------------------------------------------------------
// mount:
// ---------------------------------------------------------------------------
void ManipConfigManager::loadMountSection()
{
    const std::string ctx = "mount";
    const YAML::Node& m = require(mManipRoot, "mount", ctx);

    auto xyz = requireSeq<double>(m, "position",   ctx, 3);
    auto q   = requireSeq<double>(m, "quaternion", ctx, 4); // x y z w

    const KDL::Vector   p(xyz[0], xyz[1], xyz[2]);
    const KDL::Rotation r = KDL::Rotation::Quaternion(q[0], q[1], q[2], q[3]);
    mConfig.T_V_B = KDL::Frame(r, p);
}

// ---------------------------------------------------------------------------
// Planner: (optional top-level block) — defaults in PlannerConfig{} apply
// if absent.
// ---------------------------------------------------------------------------
void ManipConfigManager::loadPlannerSection(const YAML::Node& aTopConfig)
{
    if (!aTopConfig["Planner"] || aTopConfig["Planner"].IsNull())
        return;

    const YAML::Node& p = aTopConfig["Planner"];
    mConfig.planner.approachDistanceM = optionalScalar<double>(p, "ApproachDistanceM", mConfig.planner.approachDistanceM);
    mConfig.planner.planningTimeS     = optionalScalar<double>(p, "PlanningTimeS",     mConfig.planner.planningTimeS);
    mConfig.planner.stateValidityResolutionFrac =
        optionalScalar<double>(p, "StateValidityResolutionFrac", mConfig.planner.stateValidityResolutionFrac);
}

// ---------------------------------------------------------------------------
// Executor: (optional top-level block) — defaults in ExecutorConfig{} apply
// if absent.
// ---------------------------------------------------------------------------
void ManipConfigManager::loadExecutorSection(const YAML::Node& aTopConfig)
{
    if (!aTopConfig["Executor"] || aTopConfig["Executor"].IsNull())
        return;

    const YAML::Node& e = aTopConfig["Executor"];
    mConfig.executor.armArrivalTimeoutSec = optionalScalar<double>(e, "ArmArrivalTimeoutSec", mConfig.executor.armArrivalTimeoutSec);
    mConfig.executor.gripperDwellSec      = optionalScalar<double>(e, "GripperDwellSec",      mConfig.executor.gripperDwellSec);
    mConfig.executor.tickRateHz           = optionalScalar<double>(e, "TickRateHz",           mConfig.executor.tickRateHz);
    mConfig.executor.jointTolerance       = optionalScalar<double>(e, "JointTolerance",       mConfig.executor.jointTolerance);
    mConfig.executor.inflatedToleranceFactor =
        optionalScalar<double>(e, "InflatedToleranceFactor", mConfig.executor.inflatedToleranceFactor);
    mConfig.executor.taskPositionToleranceM = optionalScalar<double>(e, "TaskPositionToleranceM", mConfig.executor.taskPositionToleranceM);
    mConfig.executor.taskOrientationToleranceRad =
        optionalScalar<double>(e, "TaskOrientationToleranceRad", mConfig.executor.taskOrientationToleranceRad);
}

// ---------------------------------------------------------------------------
// mergeJointData — concatenate [mobile base..., arm...] into flat combined
// vectors. If there's no mobile base, the combined vectors are arm-only.
// ---------------------------------------------------------------------------
void ManipConfigManager::mergeJointData()
{
    if (mConfig.mobileBase)
    {
        append(mConfig.jointNames,       mConfig.mobileBase->jointNames);
        append(mConfig.initialPositions, mConfig.mobileBase->initialPositions);
        append(mConfig.accelLimits,      mConfig.mobileBase->accelLimits);
        append(mConfig.jerkLimits,       mConfig.mobileBase->jerkLimits);
    }

    append(mConfig.jointNames,       mConfig.manipulator.jointNames);
    append(mConfig.initialPositions, mConfig.manipulator.initialPositions);
    append(mConfig.accelLimits,      mConfig.manipulator.accelLimits);
    append(mConfig.jerkLimits,       mConfig.manipulator.jerkLimits);

    mConfig.ctrlOffset = (mConfig.mobileBase && !mConfig.mobileBase->controllable)
                       ? static_cast<int>(mConfig.mobileBase->jointNames.size()) : 0;

    auto tail = [&](const auto& v) { return std::decay_t<decltype(v)>(v.begin() + mConfig.ctrlOffset, v.end()); };
    mConfig.ctrlJointNames       = tail(mConfig.jointNames);
    mConfig.ctrlInitialPositions = tail(mConfig.initialPositions);
    mConfig.ctrlAccelLimits      = tail(mConfig.accelLimits);
    mConfig.ctrlJerkLimits       = tail(mConfig.jerkLimits);
}
