#pragma once

#include "autobot_config/CommonConfigurations.h"
#include <yaml-cpp/yaml.h>

/**
 * @brief CommonConfigManager is a singleton responsible for loading the config
 *        values shared across general/GUI nodes (Heartbeat, TableView).
 */
class CommonConfigManager
{
public:
    static CommonConfigManager* getInstance()
    {
        static CommonConfigManager instance;
        return &instance;
    }

    /**
     * @brief loadConfiguration loads the config file from the autobot_bringup pkg
     *        and parses its contents. Throws if any issues occur
     */
    void loadConfiguration();

    HeartbeatConfig& getHeartbeatConfig() { return mHeartbeatConfig; }
    TableViewConfig& getTableViewConfig() { return mTableViewConfig; }

private:
    CommonConfigManager() = default;
    ~CommonConfigManager() = default;
    CommonConfigManager(const CommonConfigManager&) = delete;
    CommonConfigManager& operator=(const CommonConfigManager&) = delete;

    void parseHeartbeatConfig(const YAML::Node& aNode);
    void parseTableViewConfig(const YAML::Node& aNode);

    HeartbeatConfig mHeartbeatConfig;
    TableViewConfig mTableViewConfig;
};
