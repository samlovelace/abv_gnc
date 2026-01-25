
#include "common/ConfigurationManager.h"
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
    s << "Configuration: ";
    s << YAML::Dump(config);
    LOGD << s.str();  

    // Parse StateMachine
    mConfigurations.stateMachineConfig.mRate = config["StateMachine"]["Rate"].as<int>(); 

    // Parse Vehicle
    YAML::Node vehicleNode = config["Vehicle"];
    mConfigurations.vehicleConfig.Name = vehicleNode["Name"].as<std::string>();
    mConfigurations.vehicleConfig.Mass = vehicleNode["Mass"].as<double>();
    mConfigurations.vehicleConfig.Inertia = vehicleNode["Inertia"].as<double>();

    // Parse Controller Gains
    YAML::Node controllerNode = config["Controller"];
    mConfigurations.vehicleConfig.controllerConfig.Kp = ConfigUtils::parseVector3d(controllerNode["Kp"]);
    mConfigurations.vehicleConfig.controllerConfig.Ki = ConfigUtils::parseVector3d(controllerNode["Ki"]);
    mConfigurations.vehicleConfig.controllerConfig.Kd = ConfigUtils::parseVector3d(controllerNode["Kd"]);

    // Parse Thrusters
    mConfigurations.vehicleConfig.controllerConfig.thrusterConfig.mForce = config["Thrusters"]["Force"].as<double>(); 
    mConfigurations.vehicleConfig.controllerConfig.thrusterConfig.mMomentArm = config["Thrusters"]["MomentArm"].as<double>();

    YAML::Node thrusterNode = config["Thrusters"]["InputDiscretization"];
    mConfigurations.vehicleConfig.controllerConfig.thrusterConfig.uOn = thrusterNode["On"].as<double>();
    mConfigurations.vehicleConfig.controllerConfig.thrusterConfig.uOff = thrusterNode["Off"].as<double>();

    YAML::Node thrusterDriverNode = config["Thrusters"]["ThrusterDriver"]; 
    mConfigurations.vehicleConfig.controllerConfig.thrusterConfig.mType = thrusterDriverNode["Type"].as<std::string>(); 
    mConfigurations.vehicleConfig.controllerConfig.thrusterConfig.mGpioPins = ConfigUtils::parseIntVector(thrusterDriverNode["GPIO"]["pins"]);

    YAML::Node networkNode = config["Network"];
    mConfigurations.vehicleConfig.stateTrackerConfig.mServerIp = networkNode["Server"]["Ip"].as<std::string>();
    mConfigurations.vehicleConfig.stateTrackerConfig.mLocalIp = networkNode["Local"]["Ip"].as<std::string>();

    YAML::Node stateTrackerNode = config["StateTracker"]; 
    mConfigurations.vehicleConfig.stateTrackerConfig.mInterface = stateTrackerNode["Interface"].as<std::string>(); 
    mConfigurations.vehicleConfig.stateTrackerConfig.mRate = stateTrackerNode["Rate"].as<int>(); 
    mConfigurations.vehicleConfig.stateTrackerConfig.mRigidBodyId = stateTrackerNode["RigidBodyId"].as<int>(); 

    YAML::Node statePubNode = config["StatePublisher"]; 
    mConfigurations.vehicleConfig.statePublisherConfig.mInterface = statePubNode["Interface"].as<std::string>(); 
    mConfigurations.vehicleConfig.statePublisherConfig.mRate = statePubNode["Rate"].as<int>(); 

    return true; 
}
