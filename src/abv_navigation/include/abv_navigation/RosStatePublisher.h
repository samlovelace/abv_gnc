#ifndef ROSSTATEPUBLISHER_H
#define ROSSTATEPUBLISHER_H

#include <thread> 

#include "common/Configurations.h"
#include "common/AbvState.hpp"
#include "abv_msgs/msg/abv_state.hpp"

class RosStatePublisher
{
public:
    RosStatePublisher();
    ~RosStatePublisher();

    void publish(const AbvState& aState); 

private:
    
    std::string mTopicName; 
    abv_msgs::msg::AbvState convertToIdlMsg(const AbvState& aStateVector); 
};

#endif // ROSSTATEPUBLISHER_H
