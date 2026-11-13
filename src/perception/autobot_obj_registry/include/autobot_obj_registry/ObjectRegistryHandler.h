#ifndef OBJECT_REGISTRY_HANDLER_H
#define OBJECT_REGISTRY_HANDLER_H

#include <cstdint>
#include <map>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "autobot_msgs/msg/object_observations.hpp"
#include "autobot_msgs/msg/tracked_object.hpp"
#include "autobot_msgs/srv/get_tracked_objects.hpp"

class ObjectRegistryHandler
{
public:
    ObjectRegistryHandler();
    ~ObjectRegistryHandler();

private:
    void observationsCallback(autobot_msgs::msg::ObjectObservations::SharedPtr aMsg);
    void getTrackedObjectsCallback(const std::shared_ptr<autobot_msgs::srv::GetTrackedObjects::Request> aRequest,
                                    std::shared_ptr<autobot_msgs::srv::GetTrackedObjects::Response> aResponse);
    void tick();

    struct TrackedObjectRecord
    {
        autobot_msgs::msg::ObjectObservation mObservation;
        rclcpp::Time mLastSeen;
    };

    int32_t findMatch(const std::string& aLabel, const geometry_msgs::msg::Point& aCentroid, double aThreshold) const;
    static double distance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b);
    autobot_msgs::msg::TrackedObject toMsg(int32_t aId, const TrackedObjectRecord& aRecord) const;

private:
    std::map<int32_t, TrackedObjectRecord> mTrackedObjects;
    int32_t mNextId{1};  // ids start at 1 so 0 is a safe "no object"/"return all" sentinel

    rclcpp::TimerBase::SharedPtr mTimer;
    rclcpp::Service<autobot_msgs::srv::GetTrackedObjects>::SharedPtr mGetTrackedObjectsService;
};
#endif  // OBJECT_REGISTRY_HANDLER_H
