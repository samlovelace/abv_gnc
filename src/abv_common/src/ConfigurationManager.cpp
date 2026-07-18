
#include "abv_common/ConfigurationManager.h"
#include "plog/Log.h"
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem> 

ConfigurationManager::ConfigurationManager()
{
    // do nothing 
}

ConfigurationManager::~ConfigurationManager()
{
    // do nothing
}

void ConfigurationManager::loadConfiguration()
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

    std::string configFilePath = ament_index_cpp::get_package_share_directory("abv_bringup") + "/config/config.yaml";
    if (!std::filesystem::exists(configFilePath))
    {
        throw std::runtime_error("Missing configuration file at " + configFilePath); 
    }

    YAML::Node config = YAML::LoadFile(configFilePath); 
    std::stringstream s; 
    s << "Configuration: \n";
    s << YAML::Dump(config);
    LOGD << s.str();  

    // Parse configs 
    YAML::Node guidanceNode = config["Guidance"]; 
    parseGuidanceConfig(guidanceNode); 

    YAML::Node navigationNode = config["Navigation"]; 
    parseNavigationConfig(navigationNode); 

    YAML::Node controlNode = config["Control"];
    parseControlConfig(controlNode);

    YAML::Node heartbeatNode = config["Heartbeat"];
    parseHeartbeatConfig(heartbeatNode);

    YAML::Node tableViewNode = config["TableView"];
    parseTableViewConfig(tableViewNode);
}

void ConfigurationManager::parseGuidanceConfig(const YAML::Node& aNode)
{
    mGuidanceConfig.mStateMachineRate = aNode["StateMachine"]["Rate"].as<int>();
    mGuidanceConfig.mWaypointTimeout = aNode["StateMachine"]["WaypointTimeout"].as<double>();
    mGuidanceConfig.mWaypointTimeoutToleranceScale = aNode["StateMachine"]["WaypointTimeoutToleranceScale"].as<double>();
}

void ConfigurationManager::parseNavigationConfig(const YAML::Node& aNode)
{
    mNavigationConfig.mInterface = aNode["Interface"].as<std::string>(); 
    mNavigationConfig.mRate = aNode["Rate"].as<int>(); 
    mNavigationConfig.mRigidBodyName = aNode["RigidBodyName"].as<std::string>(); 

    mNavigationConfig.mLocalIp = aNode["Network"]["Local"]["Ip"].as<std::string>();
    mNavigationConfig.mServerIp = aNode["Network"]["Server"]["Ip"].as<std::string>();

    mNavigationConfig.mMaxDeadReckonDuration = aNode["MaxDeadReckonDuration"].as<double>(0.5);
    mNavigationConfig.mSimulateDropout = aNode["SimulateDropout"].as<bool>(false);
}

void ConfigurationManager::parseControlConfig(const YAML::Node& aNode)
{
    mControlConfig.mStateMachineRate = aNode["StateMachine"]["Rate"].as<int>();
    mControlConfig.mControlPolicyType = aNode["ControlPolicy"].as<std::string>("PID");
    mControlConfig.mPoseArrivalTol = ConfigUtils::parseVector3d(aNode["Arrival"]["Tolerance"]);
    mControlConfig.mArrivalDuration = aNode["Arrival"]["Duration"].as<double>();
    mControlConfig.mNavDataTimeout = aNode["NavDataTimeout"].as<double>(1.0);

    mControlConfig.mKp = ConfigUtils::parseVector3d(aNode["Gains"]["Kp"]);
    mControlConfig.mKi = ConfigUtils::parseVector3d(aNode["Gains"]["Ki"]);
    mControlConfig.mKd = ConfigUtils::parseVector3d(aNode["Gains"]["Kd"]);

    mControlConfig.mSchmittTriggerOn = ConfigUtils::parseVector3d(aNode["Thrusters"]["InputDiscretization"]["On"]);
    mControlConfig.mSchmittTriggerOff = ConfigUtils::parseVector3d(aNode["Thrusters"]["InputDiscretization"]["Off"]);

    mControlConfig.mThrusterAllocationStrategy = aNode["Thrusters"]["AllocationStrategy"].as<std::string>("LookupTable");

    mControlConfig.mAllocationX   = ConfigUtils::parseIntVector(aNode["Thrusters"]["Allocation"]["x"]);
    mControlConfig.mAllocationY   = ConfigUtils::parseIntVector(aNode["Thrusters"]["Allocation"]["y"]);
    mControlConfig.mAllocationYaw = ConfigUtils::parseIntVector(aNode["Thrusters"]["Allocation"]["yaw"]);
    mControlConfig.mAllocationThreshold = aNode["Thrusters"]["Allocation"]["Threshold"].as<double>(0.1);

    mControlConfig.mThrusterDriverType = aNode["Thrusters"]["ThrusterDriver"]["Type"].as<std::string>();
    if("JETGPIO" == mControlConfig.mThrusterDriverType)
    {
        // parse GPIO pins 
        mControlConfig.mGpioPins = ConfigUtils::parseIntVector(aNode["Thrusters"]["ThrusterDriver"]["GPIO"]["pins"]); 
    }

    mControlConfig.mForce = aNode["Dynamics"]["ThrusterForce"].as<double>();
    mControlConfig.mMomentArm = aNode["Dynamics"]["MomentArm"].as<double>();
    mControlConfig.mMass = aNode["Dynamics"]["Mass"].as<double>();
    mControlConfig.mInertia = aNode["Dynamics"]["Inertia"].as<double>();
}

void ConfigurationManager::parseHeartbeatConfig(const YAML::Node& aNode)
{
    mHeartbeatConfig.mRate = aNode["Rate"].as<double>();
    mHeartbeatConfig.mStaleAfter = aNode["StaleAfter"].as<double>();
    mHeartbeatConfig.mPingIntervalMs = aNode["PingIntervalMs"].as<int>();
}

void ConfigurationManager::parseTableViewConfig(const YAML::Node& aNode)
{
    mTableViewConfig.mWidth = aNode["Width"].as<double>();
    mTableViewConfig.mHeight = aNode["Height"].as<double>();
    mTableViewConfig.mRobotWidth = aNode["RobotWidth"].as<double>();
    mTableViewConfig.mRobotLength = aNode["RobotLength"].as<double>();
}



