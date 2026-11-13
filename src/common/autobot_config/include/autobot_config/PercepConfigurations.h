#pragma once

#include <array>
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>

struct ObjDetectionConfig
{
    std::string mType;        // which IDetector implementation, e.g. "yolo11-seg"
    std::string mModelPath;
    std::vector<std::string> mClassNames;  // index == class channel in the model output

    int mInputSize;
    double mConfThresh;
    double mIouThresh;
    bool mUseCuda;
};

struct ObjSegmentationConfig
{
    double mSyncSlopSec;      // ApproximateTimeSynchronizer max interval duration
    int mMinPointsPerObject;  // skip + warn if a detection's valid point count is below this
    int mMaxPointsPerObject;  // 0 = unbounded; else caps PointCloud2 size / PCA cost for huge masks

    int mMaskErosionKernelSize;      // pixels, elliptical kernel; <=1 disables erosion
    int mOutlierMeanK;                // neighbors per point for outlier stats; <=0 disables
    double mOutlierStdDevMultiplier;  // reject if meanNeighborDist > mean + this*stddev; <=0 disables
};

struct ObjRegistryConfig
{
    double mMatchDistanceThresholdM;  // meters; max centroid distance to same-label existing track to count as a match
    double mStalenessTimeoutSec;      // seconds since last_seen before a tracked object is pruned (erased)
    double mPruneRateHz;              // rate of the prune/staleness-check wall timer
};

struct CameraExtrinsicsConfig
{
    std::string mCameraFrame;  // must match the depth camera_info frame_id actually published
    std::string mWorldFrame;   // target frame object observations are transformed into

    // Fixed camera->world extrinsics, broadcast once by autobot_camera_tf and
    // consumed by autobot_obj_segmentation via tf2 to express observations in mWorldFrame.
    // Plain array, not Eigen::Vector4d - a fixed-size Eigen type here (32 bytes,
    // Eigen's forced-vectorization-alignment size class) corrupted the adjacent
    // std::string members of this struct when stored directly in the
    // PercepConfigManager singleton; this struct does no vector math, so there's
    // no reason to take on Eigen's alignment requirements for it.
    Eigen::Vector3d mTranslationXYZ;
    std::array<double, 4> mRotationQuaternionXYZW;  // [x, y, z, w]
};
