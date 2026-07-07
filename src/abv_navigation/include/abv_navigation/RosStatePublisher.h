#ifndef ROSSTATEPUBLISHER_H
#define ROSSTATEPUBLISHER_H

#include <thread>
#include <chrono>

#include "abv_common/Configurations.h"
#include "abv_common/AbvState.hpp"
#include "abv_msgs/msg/abv_state.hpp"

class RosStatePublisher
{
public:
    RosStatePublisher();
    ~RosStatePublisher();

    void publish(const AbvState& aState,
                 std::chrono::system_clock::time_point aMeasurementTime,
                 bool aValid);

private:

    std::string mTopicName;
    abv_msgs::msg::AbvState convertToIdlMsg(const AbvState& aStateVector,
                                             std::chrono::system_clock::time_point aMeasurementTime,
                                             bool aValid);
};

#endif // ROSSTATEPUBLISHER_H
