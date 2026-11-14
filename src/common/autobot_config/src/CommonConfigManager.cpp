#include "autobot_config/CommonConfigManager.h"
#include "autobot_config/Util.hpp"
#include <filesystem>

void CommonConfigManager::loadConfiguration()
{
    std::string configFilePath = util::getConfigPath("common.yaml");
    if (!std::filesystem::exists(configFilePath))
    {
        throw std::runtime_error("Missing configuration file at " + configFilePath);
    }

    YAML::Node config = YAML::LoadFile(configFilePath);

    parseHeartbeatConfig(config["Heartbeat"]);
    parseTableViewConfig(config["TableView"]);
}

void CommonConfigManager::parseHeartbeatConfig(const YAML::Node& aNode)
{
    mHeartbeatConfig.mRate = aNode["Rate"].as<double>();
    mHeartbeatConfig.mStaleAfter = aNode["StaleAfter"].as<double>();
    mHeartbeatConfig.mPingIntervalMs = aNode["PingIntervalMs"].as<int>();
}

void CommonConfigManager::parseTableViewConfig(const YAML::Node& aNode)
{
    mTableViewConfig.mWidth = aNode["Width"].as<double>();
    mTableViewConfig.mHeight = aNode["Height"].as<double>();
    mTableViewConfig.mRobotWidth = aNode["RobotWidth"].as<double>();
    mTableViewConfig.mRobotLength = aNode["RobotLength"].as<double>();
}
