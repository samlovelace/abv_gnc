#pragma once

#include "autobot_config/PercepConfigurations.h"
#include <yaml-cpp/yaml.h>

/**
 * @brief PercepConfigManager is a singleton responsible for loading the
 *        perception-domain config values (ObjDetection, ObjSegmentation,
 *        ObjRegistry, CameraExtrinsics) and providing access to the data within.
 */
class PercepConfigManager
{
public:
    static PercepConfigManager* getInstance()
    {
        static PercepConfigManager instance;
        return &instance;
    }

    /**
     * @brief loadConfiguration loads the config file from the autobot_bringup pkg
     *        and parses its contents. Throws if any issues occur
     */
    void loadConfiguration();

    ObjDetectionConfig& getObjDetectionConfig() { return mObjDetectionConfig; }
    ObjSegmentationConfig& getObjSegmentationConfig() { return mObjSegmentationConfig; }
    ObjRegistryConfig& getObjRegistryConfig() { return mObjRegistryConfig; }
    CameraExtrinsicsConfig& getCameraExtrinsicsConfig() { return mCameraExtrinsicsConfig; }

private:
    PercepConfigManager() = default;
    ~PercepConfigManager() = default;
    PercepConfigManager(const PercepConfigManager&) = delete;
    PercepConfigManager& operator=(const PercepConfigManager&) = delete;

    void parseObjDetectionConfig(const YAML::Node& aNode);
    void parseObjSegmentationConfig(const YAML::Node& aNode);
    void parseObjRegistryConfig(const YAML::Node& aNode);
    void parseCameraExtrinsicsConfig(const YAML::Node& aNode);

    ObjDetectionConfig mObjDetectionConfig;
    ObjSegmentationConfig mObjSegmentationConfig;
    ObjRegistryConfig mObjRegistryConfig;
    CameraExtrinsicsConfig mCameraExtrinsicsConfig;
};
