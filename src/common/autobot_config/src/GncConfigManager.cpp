#include "autobot_config/GncConfigManager.h"
#include "autobot_config/YamlUtil.h"
#include "plog/Log.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

using namespace autobot_config::YamlUtil;

void GncConfigManager::loadConfiguration()
{
    LOGD << R"(
        __    __   _______  __       __        ______               ___      .______   ____    ____
       |  |  |  | |   ____||  |     |  |      /  __  \             /   \     |   _  \  \   \  /   /
       |  |__|  | |  |__   |  |     |  |     |  |  |  |           /  ^  \    |  |_)  |  \   \/   /
       |   __   | |   __|  |  |     |  |     |  |  |  |          /  /_\  \   |   _  <    \      /
       |  |  |  | |  |____ |  `----.|  `----.|  `--'  |  __     /  _____  \  |  |_)  |    \    /
       |__|  |__| |_______||_______||_______| \______/  (_ )   /__/     \__\ |______/      \__/
                                                         |/
       )";

    std::string configFilePath = ament_index_cpp::get_package_share_directory("autobot_bringup") + "/config/config.yaml";
    if (!std::filesystem::exists(configFilePath))
    {
        throw std::runtime_error("Missing configuration file at " + configFilePath);
    }

    YAML::Node config = YAML::LoadFile(configFilePath);
    std::stringstream s;
    s << "Configuration: \n";
    s << YAML::Dump(config);
    LOGD << s.str();

    parseGuidanceConfig(config["Guidance"]);
    parseNavigationConfig(config["Navigation"]);
    parseControlConfig(config["Control"]);
    parseControlLawConfig(config["ControlLaw"]);
    parseThrusterConfig(config["Thrusters"]);
    parseSceneConfig(config["Scene"]);
    parseCollisionAvoidConfig(config["CollisionAvoid"]);
}

void GncConfigManager::parseGuidanceConfig(const YAML::Node& aNode)
{
    mGuidanceConfig.mStateMachineRate = aNode["StateMachine"]["Rate"].as<int>();
    mGuidanceConfig.mWaypointTimeout = aNode["StateMachine"]["WaypointTimeout"].as<double>();
    mGuidanceConfig.mWaypointTimeoutToleranceScale = aNode["StateMachine"]["WaypointTimeoutToleranceScale"].as<double>();
}

void GncConfigManager::parseNavigationConfig(const YAML::Node& aNode)
{
    mNavigationConfig.mInterface = aNode["Interface"].as<std::string>();
    mNavigationConfig.mRate = aNode["Rate"].as<int>();
    mNavigationConfig.mRigidBodyName = aNode["RigidBodyName"].as<std::string>();

    mNavigationConfig.mLocalIp = aNode["Network"]["Local"]["Ip"].as<std::string>();
    mNavigationConfig.mServerIp = aNode["Network"]["Server"]["Ip"].as<std::string>();

    mNavigationConfig.mMaxDeadReckonDuration = aNode["MaxDeadReckonDuration"].as<double>(0.5);
    mNavigationConfig.mSimulateDropout = aNode["SimulateDropout"].as<bool>(false);
}

void GncConfigManager::parseControlConfig(const YAML::Node& aNode)
{
    mControlConfig.mStateMachineRate = aNode["StateMachine"]["Rate"].as<int>();
    mControlConfig.mPoseArrivalTol = parseVector3d(aNode["Arrival"]["Tolerance"]);
    mControlConfig.mArrivalDuration = aNode["Arrival"]["Duration"].as<double>();
    mControlConfig.mNavDataTimeout = aNode["NavDataTimeout"].as<double>(1.0);

    mControlConfig.mForce = aNode["Dynamics"]["ThrusterForce"].as<double>();
    mControlConfig.mMomentArm = aNode["Dynamics"]["MomentArm"].as<double>();
    mControlConfig.mMass = aNode["Dynamics"]["Mass"].as<double>();
    mControlConfig.mInertia = aNode["Dynamics"]["Inertia"].as<double>();

    std::string propagationMode = aNode["Dynamics"]["PropagationMode"].as<std::string>("internal");
    mControlConfig.mUseExternalPropagation = ("external" == propagationMode);
    mControlConfig.mExternalStateTopic = aNode["Dynamics"]["ExternalStateTopic"].as<std::string>("abv/sim/gazebo_state");
    mControlConfig.mWrenchCommandTopic = aNode["Dynamics"]["WrenchCommandTopic"].as<std::string>("gazebo/wrench_cmd");
    mControlConfig.mWrenchTargetLink = aNode["Dynamics"]["WrenchTargetLink"].as<std::string>("base_link");
}

void GncConfigManager::parseControlLawConfig(const YAML::Node& aNode)
{
    mControlLawConfig.mLawType = aNode["Law"].as<std::string>("PID");

    mControlLawConfig.mKp = parseVector3d(aNode["Pid"]["Kp"]);
    mControlLawConfig.mKi = parseVector3d(aNode["Pid"]["Ki"]);
    mControlLawConfig.mKd = parseVector3d(aNode["Pid"]["Kd"]);
}

void GncConfigManager::parseThrusterConfig(const YAML::Node& aNode)
{
    mThrusterConfig.mSchmittTriggerOn = parseVector3d(aNode["InputDiscretization"]["On"]);
    mThrusterConfig.mSchmittTriggerOff = parseVector3d(aNode["InputDiscretization"]["Off"]);

    mThrusterConfig.mThrusterAllocationStrategy = aNode["AllocationStrategy"].as<std::string>("LookupTable");

    mThrusterConfig.mAllocationX   = parseIntVector(aNode["Allocation"]["x"]);
    mThrusterConfig.mAllocationY   = parseIntVector(aNode["Allocation"]["y"]);
    mThrusterConfig.mAllocationYaw = parseIntVector(aNode["Allocation"]["yaw"]);
    mThrusterConfig.mAllocationThreshold = aNode["Allocation"]["Threshold"].as<double>(0.1);

    mThrusterConfig.mThrusterDriverType = aNode["ThrusterDriver"]["Type"].as<std::string>();
    if ("JETGPIO" == mThrusterConfig.mThrusterDriverType)
    {
        mThrusterConfig.mGpioPins = parseIntVector(aNode["ThrusterDriver"]["GPIO"]["pins"]);
    }

    mThrusterConfig.mForce = aNode["Force"].as<double>();
    mThrusterConfig.mMomentArm = aNode["MomentArm"].as<double>();
}

void GncConfigManager::parseSceneConfig(const YAML::Node& aNode)
{
    mSceneConfig.mXMin = aNode["XMin"].as<double>();
    mSceneConfig.mXMax = aNode["XMax"].as<double>();
    mSceneConfig.mYMin = aNode["YMin"].as<double>();
    mSceneConfig.mYMax = aNode["YMax"].as<double>();
}

void GncConfigManager::parseCollisionAvoidConfig(const YAML::Node& aNode)
{
    mCollisionAvoidConfig.mGridResolution = aNode["GridResolution"].as<double>();
    mCollisionAvoidConfig.mReplanRate = aNode["ReplanRate"].as<double>();
}
