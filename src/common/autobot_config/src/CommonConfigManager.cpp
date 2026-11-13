#include "autobot_config/CommonConfigManager.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

void CommonConfigManager::loadConfiguration()
{
    std::string configFilePath = ament_index_cpp::get_package_share_directory("autobot_bringup") + "/config/config.yaml";
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
