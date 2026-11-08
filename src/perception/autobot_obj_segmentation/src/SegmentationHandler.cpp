#include "autobot_obj_segmentation/SegmentationHandler.h"

#include <array>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <plog/Log.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "autobot_common/ConfigurationManager.h"
#include "autobot_common/RosTopicManager.h"
#include "autobot_msgs/msg/object_observation.hpp"

namespace {

constexpr const char* kDepthTopic = "/camera/camera/aligned_depth_to_color/image_raw";
constexpr const char* kCameraInfoTopic = "/camera/camera/aligned_depth_to_color/camera_info";
constexpr const char* kDetectionsTopic = "/detections";
constexpr const char* kOutputTopic = "/percep/object_observations";

// Mask is already full original-image size (same as depth), NOT cropped to
// the box - confirmed via Detection.hpp / YoloSegDetector::postprocess. So
// mask pixel (u,v) maps directly to depth pixel (u,v), no box offset needed.
bool backProject(const cv::Mat& mask, const cv::Mat& depth, const std::string& depthEncoding,
                  float fx, float fy, float cx, float cy, int maxPoints,
                  std::vector<Eigen::Vector3f>& outPoints)
{
    outPoints.clear();
    for (int v = 0; v < mask.rows; ++v)
    {
        const uint8_t* maskRow = mask.ptr<uint8_t>(v);
        for (int u = 0; u < mask.cols; ++u)
        {
            if (maskRow[u] == 0) continue;

            float z;
            if (depthEncoding == "16UC1")
            {
                const uint16_t raw = depth.at<uint16_t>(v, u);
                if (raw == 0) continue;
                z = static_cast<float>(raw) / 1000.0f;  // mm -> m
            }
            else  // "32FC1"
            {
                const float raw = depth.at<float>(v, u);
                if (raw == 0.f || !std::isfinite(raw)) continue;
                z = raw;
            }

            outPoints.emplace_back((u - cx) * z / fx, (v - cy) * z / fy, z);
            if (maxPoints > 0 && static_cast<int>(outPoints.size()) >= maxPoints) return true;
        }
    }
    return true;
}

// Centroid + covariance eigendecomposition -> dimensions (extent along each
// principal axis) + the principal axes themselves
void computePca(const std::vector<Eigen::Vector3f>& points,
                 geometry_msgs::msg::Point& centroidOut,
                 geometry_msgs::msg::Vector3& dimsOut,
                 std::array<geometry_msgs::msg::Vector3, 3>& axesOut)
{
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto& p : points) centroid += p.cast<double>();
    centroid /= static_cast<double>(points.size());

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& p : points)
    {
        const Eigen::Vector3d d = p.cast<double>() - centroid;
        cov += d * d.transpose();
    }
    cov /= static_cast<double>(points.size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    Eigen::Matrix3d axes = solver.eigenvectors();  // ascending eigenvalue order
    Eigen::Matrix3d sortedAxes;
    sortedAxes.col(0) = axes.col(2);  // largest-extent axis first
    sortedAxes.col(1) = axes.col(1);
    sortedAxes.col(2) = axes.col(0);

    Eigen::Vector3d minProj = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
    Eigen::Vector3d maxProj = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());
    for (const auto& p : points)
    {
        const Eigen::Vector3d local = sortedAxes.transpose() * (p.cast<double>() - centroid);
        minProj = minProj.cwiseMin(local);
        maxProj = maxProj.cwiseMax(local);
    }
    const Eigen::Vector3d extent = maxProj - minProj;

    centroidOut.x = centroid.x(); centroidOut.y = centroid.y(); centroidOut.z = centroid.z();
    dimsOut.x = extent.x(); dimsOut.y = extent.y(); dimsOut.z = extent.z();
    for (int i = 0; i < 3; ++i)
    {
        axesOut[i].x = sortedAxes.col(i).x();
        axesOut[i].y = sortedAxes.col(i).y();
        axesOut[i].z = sortedAxes.col(i).z();
    }
}

sensor_msgs::msg::PointCloud2 buildPointCloud(const std::vector<Eigen::Vector3f>& points,
                                               const std_msgs::msg::Header& header)
{
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(points.size());
    cloud.is_dense = true;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(cloud, "z");
    for (const auto& p : points)
    {
        *ix = p.x(); *iy = p.y(); *iz = p.z();
        ++ix; ++iy; ++iz;
    }
    return cloud;
}

cv::Mat depthMsgToMat(const sensor_msgs::msg::Image& msg)
{
    if (msg.encoding == "16UC1")
        return cv::Mat(msg.height, msg.width, CV_16UC1, const_cast<uint8_t*>(msg.data.data()), msg.step);
    if (msg.encoding == "32FC1")
        return cv::Mat(msg.height, msg.width, CV_32FC1, const_cast<uint8_t*>(msg.data.data()), msg.step);
    throw std::runtime_error("unsupported depth encoding: " + msg.encoding);
}

cv::Mat maskMsgToMat(const sensor_msgs::msg::Image& msg)
{
    if (msg.data.empty()) return cv::Mat();  // box-only detection, no mask
    return cv::Mat(msg.height, msg.width, CV_8UC1, const_cast<uint8_t*>(msg.data.data()), msg.step);
}

}  // namespace

SegmentationHandler::SegmentationHandler()
{
    RosTopicManager::getInstance()->createSubscriber<sensor_msgs::msg::CameraInfo>(
        kCameraInfoTopic, std::bind(&SegmentationHandler::onCameraInfo, this, std::placeholders::_1));
    RosTopicManager::getInstance()->createPublisher<autobot_msgs::msg::ObjectObservations>(kOutputTopic);

    mDepthSub.subscribe(RosTopicManager::getInstance(), kDepthTopic, rmw_qos_profile_sensor_data);
    mDetectionsSub.subscribe(RosTopicManager::getInstance(), kDetectionsTopic, rmw_qos_profile_default);

    const auto& cfg = ConfigurationManager::getInstance()->getObjSegmentationConfig();

    mSync = std::make_unique<Synchronizer>(SyncPolicy(10), mDepthSub, mDetectionsSub);
    mSync->setMaxIntervalDuration(rclcpp::Duration::from_seconds(cfg.mSyncSlopSec));
    mSync->registerCallback(std::bind(&SegmentationHandler::onSynced, this,
                                       std::placeholders::_1, std::placeholders::_2));
}

SegmentationHandler::~SegmentationHandler() = default;

void SegmentationHandler::onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr aInfo)
{
    std::lock_guard<std::mutex> lk(mCameraInfoMutex);
    mLatestCameraInfo = aInfo;
}

void SegmentationHandler::onSynced(const sensor_msgs::msg::Image::ConstSharedPtr& aDepth,
                                    const autobot_msgs::msg::Detections2D::ConstSharedPtr& aDetections)
{
    sensor_msgs::msg::CameraInfo::SharedPtr info;
    {
        std::lock_guard<std::mutex> lk(mCameraInfoMutex);
        info = mLatestCameraInfo;
    }
    if (!info)
    {
        LOGW << "No CameraInfo cached yet, dropping synced frame";
        return;
    }

    const auto& cfg = ConfigurationManager::getInstance()->getObjSegmentationConfig();
    const float fx = static_cast<float>(info->k[0]);
    const float fy = static_cast<float>(info->k[4]);
    const float cx = static_cast<float>(info->k[2]);
    const float cy = static_cast<float>(info->k[5]);

    cv::Mat depthMat;
    try
    {
        depthMat = depthMsgToMat(*aDepth);
    }
    catch (const std::exception& e)
    {
        LOGE << e.what();
        return;
    }

    autobot_msgs::msg::ObjectObservations outMsg;
    outMsg.header = aDetections->header;

    for (const auto& det : aDetections->detections)
    {
        cv::Mat maskMat = maskMsgToMat(det.mask);
        if (maskMat.empty()) continue;  // box-only detection: nothing to back-project
        if (maskMat.size() != depthMat.size())
        {
            LOGE << "Mask/depth size mismatch for '" << det.label << "', skipping";
            continue;
        }

        std::vector<Eigen::Vector3f> points;
        backProject(maskMat, depthMat, aDepth->encoding, fx, fy, cx, cy, cfg.mMaxPointsPerObject, points);

        if (static_cast<int>(points.size()) < cfg.mMinPointsPerObject)
        {
            LOGW << "Detection '" << det.label << "' has only " << points.size()
                 << " valid points (< " << cfg.mMinPointsPerObject << "), skipping";
            continue;
        }

        autobot_msgs::msg::ObjectObservation obs;
        obs.label = det.label;
        obs.class_id = -1;  // no numeric class id available yet (Detection2D only carries label)
        obs.confidence = det.score;
        computePca(points, obs.centroid, obs.dimensions, obs.principal_axes);
        obs.points = buildPointCloud(points, aDetections->header);
        outMsg.observations.push_back(std::move(obs));
    }

    RosTopicManager::getInstance()->publishMessage(kOutputTopic, outMsg);
}
