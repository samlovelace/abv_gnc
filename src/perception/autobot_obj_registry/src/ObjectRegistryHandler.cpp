#include "autobot_obj_registry/ObjectRegistryHandler.h"

#include <cmath>
#include <limits>

#include <plog/Log.h>

#include "autobot_config/PercepConfigManager.h"
#include "autobot_common/RosTopicManager.h"

namespace {
constexpr const char* kObservationsTopic = "/percep/object_observations";
constexpr const char* kGetTrackedObjectsService = "percep/get_tracked_objects";
}  // namespace

ObjectRegistryHandler::ObjectRegistryHandler()
{
    auto topicManager = RosTopicManager::getInstance();

    topicManager->createSubscriber<autobot_msgs::msg::ObjectObservations>(kObservationsTopic,
                                    std::bind(&ObjectRegistryHandler::observationsCallback, this, std::placeholders::_1));

    mGetTrackedObjectsService = topicManager->create_service<autobot_msgs::srv::GetTrackedObjects>(kGetTrackedObjectsService,
                                    std::bind(&ObjectRegistryHandler::getTrackedObjectsCallback, this,
                                              std::placeholders::_1, std::placeholders::_2));

    const double rateHz = PercepConfigManager::getInstance()->getObjRegistryConfig().mPruneRateHz;
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(1.0 / rateHz));
    mTimer = topicManager->create_wall_timer(period, std::bind(&ObjectRegistryHandler::tick, this));
}

ObjectRegistryHandler::~ObjectRegistryHandler() = default;

void ObjectRegistryHandler::observationsCallback(autobot_msgs::msg::ObjectObservations::SharedPtr aMsg)
{
    const auto& cfg = PercepConfigManager::getInstance()->getObjRegistryConfig();
    const rclcpp::Time now = RosTopicManager::getInstance()->now();

    for (const auto& obs : aMsg->observations)
    {
        const int32_t matchId = findMatch(obs.label, obs.centroid, cfg.mMatchDistanceThresholdM);

        if (matchId > 0)
        {
            auto& record = mTrackedObjects[matchId];
            record.mObservation = obs;   // overwrite with latest - never averaged/fused
            record.mLastSeen = now;
        }
        else
        {
            TrackedObjectRecord record;
            record.mObservation = obs;
            record.mLastSeen = now;
            const int32_t newId = mNextId++;
            mTrackedObjects.emplace(newId, std::move(record));
            LOGD << "New tracked object id=" << newId << " label=" << obs.label;
        }
    }
}

void ObjectRegistryHandler::getTrackedObjectsCallback(const std::shared_ptr<autobot_msgs::srv::GetTrackedObjects::Request> aRequest,
                                                        std::shared_ptr<autobot_msgs::srv::GetTrackedObjects::Response> aResponse)
{
    aResponse->objects.clear();

    if (aRequest->id <= 0)
    {
        for (const auto& [id, record] : mTrackedObjects)
        {
            aResponse->objects.push_back(toMsg(id, record));
        }
        return;
    }

    auto it = mTrackedObjects.find(aRequest->id);
    if (it != mTrackedObjects.end())
    {
        aResponse->objects.push_back(toMsg(it->first, it->second));
    }
    // else: not found - leave aResponse->objects empty
}

void ObjectRegistryHandler::tick()
{
    const auto& cfg = PercepConfigManager::getInstance()->getObjRegistryConfig();
    const rclcpp::Time now = RosTopicManager::getInstance()->now();

    for (auto it = mTrackedObjects.begin(); it != mTrackedObjects.end(); )
    {
        const double age = (now - it->second.mLastSeen).seconds();
        if (age > cfg.mStalenessTimeoutSec)
        {
            LOGD << "Pruning tracked object id=" << it->first
                 << " (label=" << it->second.mObservation.label << "), stale for " << age << "s";
            it = mTrackedObjects.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

int32_t ObjectRegistryHandler::findMatch(const std::string& aLabel, const geometry_msgs::msg::Point& aCentroid, double aThreshold) const
{
    int32_t bestId = -1;
    double bestDist = std::numeric_limits<double>::max();

    for (const auto& [id, record] : mTrackedObjects)
    {
        if (record.mObservation.label != aLabel) continue;

        const double d = distance(record.mObservation.centroid, aCentroid);
        if (d <= aThreshold && d < bestDist)
        {
            bestDist = d;
            bestId = id;
        }
    }
    return bestId;
}

double ObjectRegistryHandler::distance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b)
{
    const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

autobot_msgs::msg::TrackedObject ObjectRegistryHandler::toMsg(int32_t aId, const TrackedObjectRecord& aRecord) const
{
    autobot_msgs::msg::TrackedObject msg;
    msg.id = aId;
    msg.observation = aRecord.mObservation;
    msg.last_seen = aRecord.mLastSeen;  // rclcpp::Time -> builtin_interfaces::msg::Time via its conversion operator
    return msg;
}
