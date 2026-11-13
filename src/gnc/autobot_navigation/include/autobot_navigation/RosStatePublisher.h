#ifndef ROSSTATEPUBLISHER_H
#define ROSSTATEPUBLISHER_H

#include <thread>
#include <chrono>

#include "autobot_config/GncConfigurations.h"
#include "autobot_common/State.hpp"
#include "autobot_msgs/msg/state.hpp"

class RosStatePublisher
{
public:
    RosStatePublisher();
    ~RosStatePublisher();

    void publish(const State& aState,
                 std::chrono::system_clock::time_point aMeasurementTime,
                 bool aValid);

private:

    std::string mTopicName;
    autobot_msgs::msg::State convertToIdlMsg(const State& aStateVector,
                                             std::chrono::system_clock::time_point aMeasurementTime,
                                             bool aValid);
};

#endif // ROSSTATEPUBLISHER_H
