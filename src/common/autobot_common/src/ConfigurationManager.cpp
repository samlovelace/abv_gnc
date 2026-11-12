
#include "autobot_common/ConfigurationManager.h"
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

    // Parse configs 
    YAML::Node guidanceNode = config["Guidance"]; 
    parseGuidanceConfig(guidanceNode); 

    YAML::Node navigationNode = config["Navigation"]; 
    parseNavigationConfig(navigationNode); 

    YAML::Node controlNode = config["Control"];
    parseControlConfig(controlNode);

    YAML::Node controlLawNode = config["ControlLaw"];
    parseControlLawConfig(controlLawNode);

    YAML::Node thrustersNode = config["Thrusters"];
    parseThrusterConfig(thrustersNode);

    YAML::Node heartbeatNode = config["Heartbeat"];
    parseHeartbeatConfig(heartbeatNode);

    YAML::Node tableViewNode = config["TableView"];
    parseTableViewConfig(tableViewNode);

    YAML::Node sceneNode = config["Scene"];
    parseSceneConfig(sceneNode);

    YAML::Node collisionAvoidNode = config["CollisionAvoid"];
    parseCollisionAvoidConfig(collisionAvoidNode);

    YAML::Node objDetectionNode = config["ObjDetection"];
    parseObjDetectionConfig(objDetectionNode);

    YAML::Node objSegmentationNode = config["ObjSegmentation"];
    parseObjSegmentationConfig(objSegmentationNode);

    YAML::Node objRegistryNode = config["ObjRegistry"];
    parseObjRegistryConfig(objRegistryNode);

    YAML::Node cameraExtrinsicsNode = config["CameraExtrinsics"];
    parseCameraExtrinsicsConfig(cameraExtrinsicsNode);
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
    mControlConfig.mPoseArrivalTol = ConfigUtils::parseVector3d(aNode["Arrival"]["Tolerance"]);
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

void ConfigurationManager::parseControlLawConfig(const YAML::Node& aNode)
{
    mControlLawConfig.mLawType = aNode["Law"].as<std::string>("PID");

    mControlLawConfig.mKp = ConfigUtils::parseVector3d(aNode["Pid"]["Kp"]);
    mControlLawConfig.mKi = ConfigUtils::parseVector3d(aNode["Pid"]["Ki"]);
    mControlLawConfig.mKd = ConfigUtils::parseVector3d(aNode["Pid"]["Kd"]);
}

void ConfigurationManager::parseThrusterConfig(const YAML::Node& aNode)
{
    mThrusterConfig.mSchmittTriggerOn = ConfigUtils::parseVector3d(aNode["InputDiscretization"]["On"]);
    mThrusterConfig.mSchmittTriggerOff = ConfigUtils::parseVector3d(aNode["InputDiscretization"]["Off"]);

    mThrusterConfig.mThrusterAllocationStrategy = aNode["AllocationStrategy"].as<std::string>("LookupTable");

    mThrusterConfig.mAllocationX   = ConfigUtils::parseIntVector(aNode["Allocation"]["x"]);
    mThrusterConfig.mAllocationY   = ConfigUtils::parseIntVector(aNode["Allocation"]["y"]);
    mThrusterConfig.mAllocationYaw = ConfigUtils::parseIntVector(aNode["Allocation"]["yaw"]);
    mThrusterConfig.mAllocationThreshold = aNode["Allocation"]["Threshold"].as<double>(0.1);

    mThrusterConfig.mThrusterDriverType = aNode["ThrusterDriver"]["Type"].as<std::string>();
    if("JETGPIO" == mThrusterConfig.mThrusterDriverType)
    {
        // parse GPIO pins
        mThrusterConfig.mGpioPins = ConfigUtils::parseIntVector(aNode["ThrusterDriver"]["GPIO"]["pins"]);
    }

    mThrusterConfig.mForce = aNode["Force"].as<double>();
    mThrusterConfig.mMomentArm = aNode["MomentArm"].as<double>();
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

void ConfigurationManager::parseSceneConfig(const YAML::Node& aNode)
{
    mSceneConfig.mXMin = aNode["XMin"].as<double>();
    mSceneConfig.mXMax = aNode["XMax"].as<double>();
    mSceneConfig.mYMin = aNode["YMin"].as<double>();
    mSceneConfig.mYMax = aNode["YMax"].as<double>();
}

void ConfigurationManager::parseCollisionAvoidConfig(const YAML::Node& aNode)
{
    mCollisionAvoidConfig.mGridResolution = aNode["GridResolution"].as<double>();
    mCollisionAvoidConfig.mReplanRate = aNode["ReplanRate"].as<double>();
}

void ConfigurationManager::parseObjDetectionConfig(const YAML::Node& aNode)
{
    mObjDetectionConfig.mType = aNode["Type"].as<std::string>();
    mObjDetectionConfig.mModelPath = aNode["ModelPath"].as<std::string>();
    mObjDetectionConfig.mClassNames = aNode["ClassNames"].as<std::vector<std::string>>();

    mObjDetectionConfig.mInputSize = aNode["InputSize"].as<int>(640);
    mObjDetectionConfig.mConfThresh = aNode["ConfThresh"].as<double>(0.25);
    mObjDetectionConfig.mIouThresh = aNode["IouThresh"].as<double>(0.45);
    mObjDetectionConfig.mUseCuda = aNode["UseCuda"].as<bool>(false);
}

void ConfigurationManager::parseObjSegmentationConfig(const YAML::Node& aNode)
{
    mObjSegmentationConfig.mSyncSlopSec = aNode["SyncSlopSec"].as<double>(0.05);
    mObjSegmentationConfig.mMinPointsPerObject = aNode["MinPointsPerObject"].as<int>(20);
    mObjSegmentationConfig.mMaxPointsPerObject = aNode["MaxPointsPerObject"].as<int>(0);

    mObjSegmentationConfig.mMaskErosionKernelSize = aNode["MaskErosionKernelSize"].as<int>(5);
    mObjSegmentationConfig.mOutlierMeanK = aNode["OutlierMeanK"].as<int>(8);
    mObjSegmentationConfig.mOutlierStdDevMultiplier = aNode["OutlierStdDevMultiplier"].as<double>(1.0);
}

void ConfigurationManager::parseObjRegistryConfig(const YAML::Node& aNode)
{
    mObjRegistryConfig.mMatchDistanceThresholdM = aNode["MatchDistanceThresholdM"].as<double>(0.5);
    mObjRegistryConfig.mStalenessTimeoutSec = aNode["StalenessTimeoutSec"].as<double>(10.0);
    mObjRegistryConfig.mPruneRateHz = aNode["PruneRateHz"].as<double>(1.0);
}

void ConfigurationManager::parseCameraExtrinsicsConfig(const YAML::Node& aNode)
{
    mCameraExtrinsicsConfig.mCameraFrame = aNode["CameraFrame"].as<std::string>("overhead_camera");
    mCameraExtrinsicsConfig.mWorldFrame = aNode["WorldFrame"].as<std::string>("world");
    mCameraExtrinsicsConfig.mTranslationXYZ = ConfigUtils::parseVector3d(aNode["TranslationXYZ"]);

    const YAML::Node& quatNode = aNode["RotationQuaternionXYZW"];
    if (quatNode && quatNode.IsSequence() && quatNode.size() == 4)
    {
        mCameraExtrinsicsConfig.mRotationQuaternionXYZW = {
            quatNode[0].as<double>(), quatNode[1].as<double>(), quatNode[2].as<double>(), quatNode[3].as<double>()};
    }
    else
    {
        throw std::runtime_error("Invalid RotationQuaternionXYZW format in YAML.");
    }
}


