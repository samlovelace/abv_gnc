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
     * @return true if the config file was parsed successfully, false otherwise. 
     */
    bool loadConfiguration(const std::string& aFilePath);

    /**
     * @brief Getters for various sub-configs
     */
    GuidanceConfig& getGuidanceConfig() { return mGuidanceConfig; }
    NavigationConfig& getNavigationConfig() { return mNavigationConfig; }
    ControlConfig& getControlConfig() { return mControlConfig; }

private: 
    /**
     * @brief private constructor. Does nothing.  
     */
    ConfigurationManager();
    /**
     * @brief private destructor. Does nothing.
     */
    ~ConfigurationManager();

    void parseGuidanceConfig(const YAML::Node& aNode); 
    void parseNavigationConfig(const YAML::Node& aNode); 
    void parseControlConfig(const YAML::Node& aNode); 

private: 

    GuidanceConfig mGuidanceConfig; 
    NavigationConfig mNavigationConfig; 
    ControlConfig mControlConfig; 
};

#endif

    