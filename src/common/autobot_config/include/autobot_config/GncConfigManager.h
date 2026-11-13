#pragma once

#include "autobot_config/GncConfigurations.h"
#include <yaml-cpp/yaml.h>

/**
 * @brief GncConfigManager is a singleton responsible for loading the GNC-domain
 *        config values (Guidance, Navigation, Control, ControlLaw, Thrusters,
 *        Scene, CollisionAvoid) and providing access to the data within.
 */
class GncConfigManager
{
public:
    static GncConfigManager* getInstance()
    {
        static GncConfigManager instance;
        return &instance;
    }

    /**
     * @brief loadConfiguration loads the config file from the autobot_bringup pkg
     *        and parses its contents. Throws if any issues occur
     */
    void loadConfiguration();

    GuidanceConfig& getGuidanceConfig() { return mGuidanceConfig; }
    NavigationConfig& getNavigationConfig() { return mNavigationConfig; }
    ControlConfig& getControlConfig() { return mControlConfig; }
    ControlLawConfig& getControlLawConfig() { return mControlLawConfig; }
    ThrusterConfig& getThrusterConfig() { return mThrusterConfig; }
    SceneConfig& getSceneConfig() { return mSceneConfig; }
    CollisionAvoidConfig& getCollisionAvoidConfig() { return mCollisionAvoidConfig; }

private:
    GncConfigManager() = default;
    ~GncConfigManager() = default;
    GncConfigManager(const GncConfigManager&) = delete;
    GncConfigManager& operator=(const GncConfigManager&) = delete;

    void parseGuidanceConfig(const YAML::Node& aNode);
    void parseNavigationConfig(const YAML::Node& aNode);
    void parseControlConfig(const YAML::Node& aNode);
    void parseControlLawConfig(const YAML::Node& aNode);
    void parseThrusterConfig(const YAML::Node& aNode);
    void parseSceneConfig(const YAML::Node& aNode);
    void parseCollisionAvoidConfig(const YAML::Node& aNode);

    GuidanceConfig mGuidanceConfig;
    NavigationConfig mNavigationConfig;
    ControlConfig mControlConfig;
    ControlLawConfig mControlLawConfig;
    ThrusterConfig mThrusterConfig;
    SceneConfig mSceneConfig;
    CollisionAvoidConfig mCollisionAvoidConfig;
};
