
#include "abv_common/ConfigurationManager.h"
#include "plog/Log.h"
#include <yaml-cpp/yaml.h>

ConfigurationManager::ConfigurationManager()
{
    // do nothing 
}

ConfigurationManager::~ConfigurationManager()
{
    // do nothing
}

bool ConfigurationManager::loadConfiguration(const std::string& aFilePath)
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
   
    YAML::Node config = YAML::LoadFile(aFilePath); 
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
    
    return true; 
}

void ConfigurationManager::parseGuidanceConfig(const YAML::Node& aNode)
{
    mGuidanceConfig.mStateMachineRate = aNode["StateMachine"]["Rate"].as<int>(); 
}

void ConfigurationManager::parseNavigationConfig(const YAML::Node& aNode)
{
    mNavigationConfig.mInterface = aNode["Interface"].as<std::string>(); 
    mNavigationConfig.mRate = aNode["Rate"].as<int>(); 
    mNavigationConfig.mRigidBodyName = aNode["RigidBodyName"].as<std::string>(); 

    mNavigationConfig.mLocalIp = aNode["Network"]["Local"]["Ip"].as<std::string>();
    mNavigationConfig.mServerIp = aNode["Network"]["Server"]["Ip"].as<std::string>(); 
}

void ConfigurationManager::parseControlConfig(const YAML::Node& aNode)
{
    mControlConfig.mStateMachineRate = aNode["StateMachine"]["Rate"].as<int>(); 
    mControlConfig.mPoseArrivalTol = ConfigUtils::parseVector3d(aNode["Arrival"]["Tolerance"]);
    mControlConfig.mArrivalDuration = aNode["Arrival"]["Duration"].as<double>(); 

    mControlConfig.mKp = ConfigUtils::parseVector3d(aNode["Gains"]["Kp"]);
    mControlConfig.mKi = ConfigUtils::parseVector3d(aNode["Gains"]["Ki"]);
    mControlConfig.mKd = ConfigUtils::parseVector3d(aNode["Gains"]["Kd"]);

    mControlConfig.mSchmittTriggerOn = aNode["Thrusters"]["InputDiscretization"]["On"].as<double>(); 
    mControlConfig.mSchmittTriggerOff = aNode["Thrusters"]["InputDiscretization"]["Off"].as<double>();

    mControlConfig.mThrusterDriverType = aNode["Thrusters"]["ThrusterDriver"]["Type"].as<std::string>(); 
    if("JETGPIO" == mControlConfig.mThrusterDriverType)
    {
        // parse GPIO pins 
        mControlConfig.mGpioPins = ConfigUtils::parseIntVector(aNode["Thrusters"]["ThrusterDriver"]["GPIO"]["pins"]); 
    }

    mControlConfig.mForce = aNode["Dynamics"]["ThrusterForce"].as<double>(); 
    mControlConfig.mMomentArm = aNode["Dynamics"]["MomentArm"].as<double>(); 
}



