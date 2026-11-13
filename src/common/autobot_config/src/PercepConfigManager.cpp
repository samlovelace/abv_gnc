#include "autobot_config/PercepConfigManager.h"
#include "autobot_config/YamlUtil.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

using namespace autobot_config::YamlUtil;

void PercepConfigManager::loadConfiguration()
{
    std::string configFilePath = ament_index_cpp::get_package_share_directory("autobot_bringup") + "/config/config.yaml";
    if (!std::filesystem::exists(configFilePath))
    {
        throw std::runtime_error("Missing configuration file at " + configFilePath);
    }

    YAML::Node config = YAML::LoadFile(configFilePath);

    parseObjDetectionConfig(config["ObjDetection"]);
    parseObjSegmentationConfig(config["ObjSegmentation"]);
    parseObjRegistryConfig(config["ObjRegistry"]);
    parseCameraExtrinsicsConfig(config["CameraExtrinsics"]);
}

void PercepConfigManager::parseObjDetectionConfig(const YAML::Node& aNode)
{
    mObjDetectionConfig.mType = aNode["Type"].as<std::string>();
    mObjDetectionConfig.mModelPath = aNode["ModelPath"].as<std::string>();
    mObjDetectionConfig.mClassNames = aNode["ClassNames"].as<std::vector<std::string>>();

    mObjDetectionConfig.mInputSize = aNode["InputSize"].as<int>(640);
    mObjDetectionConfig.mConfThresh = aNode["ConfThresh"].as<double>(0.25);
    mObjDetectionConfig.mIouThresh = aNode["IouThresh"].as<double>(0.45);
    mObjDetectionConfig.mUseCuda = aNode["UseCuda"].as<bool>(false);
}

void PercepConfigManager::parseObjSegmentationConfig(const YAML::Node& aNode)
{
    mObjSegmentationConfig.mSyncSlopSec = aNode["SyncSlopSec"].as<double>(0.05);
    mObjSegmentationConfig.mMinPointsPerObject = aNode["MinPointsPerObject"].as<int>(20);
    mObjSegmentationConfig.mMaxPointsPerObject = aNode["MaxPointsPerObject"].as<int>(0);

    mObjSegmentationConfig.mMaskErosionKernelSize = aNode["MaskErosionKernelSize"].as<int>(5);
    mObjSegmentationConfig.mOutlierMeanK = aNode["OutlierMeanK"].as<int>(8);
    mObjSegmentationConfig.mOutlierStdDevMultiplier = aNode["OutlierStdDevMultiplier"].as<double>(1.0);
}

void PercepConfigManager::parseObjRegistryConfig(const YAML::Node& aNode)
{
    mObjRegistryConfig.mMatchDistanceThresholdM = aNode["MatchDistanceThresholdM"].as<double>(0.5);
    mObjRegistryConfig.mStalenessTimeoutSec = aNode["StalenessTimeoutSec"].as<double>(10.0);
    mObjRegistryConfig.mPruneRateHz = aNode["PruneRateHz"].as<double>(1.0);
}

void PercepConfigManager::parseCameraExtrinsicsConfig(const YAML::Node& aNode)
{
    mCameraExtrinsicsConfig.mCameraFrame = aNode["CameraFrame"].as<std::string>("overhead_camera");
    mCameraExtrinsicsConfig.mWorldFrame = aNode["WorldFrame"].as<std::string>("world");
    mCameraExtrinsicsConfig.mTranslationXYZ = parseVector3d(aNode["TranslationXYZ"]);

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
