#include "autobot_obj_segmentation/SegmentationHandler.h"

#include <array>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <plog/Log.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>

#include "autobot_common/ConfigurationManager.h"
#include "autobot_common/RosTopicManager.h"
#include "autobot_msgs/msg/object_observation.hpp"

namespace {

constexpr const char* kDepthTopic = "/camera/camera/aligned_depth_to_color/image_raw";
constexpr const char* kCameraInfoTopic = "/camera/camera/aligned_depth_to_color/camera_info";
constexpr const char* kDetectionsTopic = "percep/detections";
constexpr const char* kOutputTopic = "/percep/object_observations";

// Shrinks the mask inward before back-projection so only confidently-interior
// pixels are used - the thresholded YOLO-seg mask boundary is imprecise by a
// few pixels (upsampled from a low-resolution prototype), and those boundary
// pixels often sample background depth rather than object depth. Elliptical
// kernel to avoid a directional/blocky bias against round objects.
cv::Mat erodeMask(const cv::Mat& mask, int kernelSize)
{
    if (kernelSize <= 1) return mask;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernelSize, kernelSize));
    cv::Mat eroded;
    cv::erode(mask, eroded, kernel);
    return eroded;
}

// Mask is already full original-image size (same as depth), NOT cropped to
// the box - confirmed via Detection.hpp / YoloSegDetector::postprocess. So
// mask pixel (u,v) maps directly to depth pixel (u,v), no box offset needed.
bool backProject(const cv::Mat& mask, const cv::Mat& depth, const std::string& depthEncoding,
                  float fx, float fy, float cx, float cy, int maxPoints,
                  pcl::PointCloud<pcl::PointXYZ>& outCloud)
{
    outCloud.clear();
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

            outCloud.emplace_back((u - cx) * z / fx, (v - cy) * z / fy, z);
            if (maxPoints > 0 && static_cast<int>(outCloud.size()) >= maxPoints) return true;
        }
    }
    return true;
}

// General-purpose statistical outlier removal (PCL): for each point, mean
// distance to its K nearest neighbors; reject points whose mean neighbor
// distance is more than stdDevMultiplier standard deviations above the
// cloud's own mean. Only the upper tail is rejected - points closer together
// than average are never removed, since density alone isn't evidence of
// being wrong. Filters in place.
void rejectStatisticalOutliers(pcl::PointCloud<pcl::PointXYZ>& cloud, int meanK, double stdDevMultiplier)
{
    if (meanK <= 0 || stdDevMultiplier <= 0.0 || static_cast<int>(cloud.size()) <= meanK) return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>(cloud));
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(input);
    sor.setMeanK(meanK);
    sor.setStddevMulThresh(stdDevMultiplier);
    sor.filter(cloud);
}

// Centroid + covariance eigendecomposition -> dimensions (extent along each
// principal axis) + the principal axes themselves. Centroid/covariance come
// from PCL; the eigenvalue-sorted-axis + extent-projection logic has no PCL
// equivalent (specific to what this message needs), so it stays hand-rolled.
void computePca(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                 geometry_msgs::msg::Point& centroidOut,
                 geometry_msgs::msg::Vector3& dimsOut,
                 std::array<geometry_msgs::msg::Vector3, 3>& axesOut)
{
    Eigen::Vector4f centroid4;
    Eigen::Matrix3f covariance;
    pcl::computeMeanAndCovarianceMatrix(cloud, covariance, centroid4);
    const Eigen::Vector3d centroid = centroid4.head<3>().cast<double>();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    const Eigen::Matrix3f axes = solver.eigenvectors();  // ascending eigenvalue order
    Eigen::Matrix3d sortedAxes;
    sortedAxes.col(0) = axes.col(2).cast<double>();  // largest-extent axis first
    sortedAxes.col(1) = axes.col(1).cast<double>();
    sortedAxes.col(2) = axes.col(0).cast<double>();

    Eigen::Vector3d minProj = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
    Eigen::Vector3d maxProj = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());
    for (const auto& p : cloud)
    {
        const Eigen::Vector3d local = sortedAxes.transpose() * (Eigen::Vector3d(p.x, p.y, p.z) - centroid);
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

sensor_msgs::msg::PointCloud2 buildPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                               const std_msgs::msg::Header& header)
{
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header = header;
    return msg;
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

        cv::Mat erodedMask = erodeMask(maskMat, cfg.mMaskErosionKernelSize);

        pcl::PointCloud<pcl::PointXYZ> cloud;
        backProject(erodedMask, depthMat, aDepth->encoding, fx, fy, cx, cy, cfg.mMaxPointsPerObject, cloud);

        rejectStatisticalOutliers(cloud, cfg.mOutlierMeanK, cfg.mOutlierStdDevMultiplier);

        if (static_cast<int>(cloud.size()) < cfg.mMinPointsPerObject)
        {
            LOGW << "Detection '" << det.label << "' has only " << cloud.size()
                 << " valid points (< " << cfg.mMinPointsPerObject << "), skipping";
            continue;
        }

        autobot_msgs::msg::ObjectObservation obs;
        obs.label = det.label;
        obs.class_id = -1;  // no numeric class id available yet (Detection2D only carries label)
        obs.confidence = det.score;
        computePca(cloud, obs.centroid, obs.dimensions, obs.principal_axes);
        obs.points = buildPointCloud(cloud, aDetections->header);
        outMsg.observations.push_back(std::move(obs));
    }

    RosTopicManager::getInstance()->publishMessage(kOutputTopic, outMsg);
}
