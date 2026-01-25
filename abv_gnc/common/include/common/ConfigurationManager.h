#ifndef CONFIGURATIONMANAGER_H
#define CONFIGURATIONMANAGER_H

#include "Configurations.h"
#include <yaml-cpp/yaml.h>

/**
 * @brief ConfigurationManager is a singleton responsible for loading the YAML config file and providing access
 *        to the data within it. 
 */
class ConfigurationManager
{
public:

    /**
     * @brief getInstance is the function used to obtain the singleton instance. 
     * 
     * @return a pointer to the singleton instance.
     */
    static ConfigurationManager* getInstance()
    {
        static ConfigurationManager instance;
        return &instance;
    }

    /**
     * @brief loadConfiguration loads the config file and parsing its contents. 
     * 
     * @param aFilePath the path ot the config file to load and parse. 
     * 
     * @return true if the config file was parsed successfully, false otherwise. 
     */
    bool loadConfiguration(const std::string& aFilePath);

    /**
     * @brief Getters for various sub-configs
     */
    ThrusterConfig getThrusterConfig() { return mConfigurations.vehicleConfig.controllerConfig.thrusterConfig; }
    StateTrackerConfig getStateTrackerConfig() {return mConfigurations.vehicleConfig.stateTrackerConfig;}
    StatePublisherConfig getStatePublisherConfig() {return mConfigurations.vehicleConfig.statePublisherConfig;}
    ControllerConfig getControllerConfig() {return mConfigurations.vehicleConfig.controllerConfig;}
    VehicleConfig getVehicleConfig() {return mConfigurations.vehicleConfig; }
    StateMachineConfig getStateMachineConfig() {return mConfigurations.stateMachineConfig;}

private: 
    /**
     * @brief private constructor. Does nothing.  
     */
    ConfigurationManager();
    /**
     * @brief private destructor. Does nothing.
     */
    ~ConfigurationManager();

private: 
    Configurations mConfigurations;
};

#endif

    