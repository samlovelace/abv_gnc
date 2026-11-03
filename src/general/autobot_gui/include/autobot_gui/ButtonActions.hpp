#ifndef BUTTONACTIONS_HPP
#define BUTTONACTIONS_HPP

#include "autobot_common/RosTopicManager.h"
#include "autobot_msgs/msg/controller_command.hpp"

namespace btn
{
    namespace action
    {
        inline void stop()
        {
            autobot_msgs::msg::ControllerCommand cmd; 
            cmd.set__type("STOP"); 

            RosTopicManager::getInstance()->publishMessage("abv/controller/command", cmd); 
        }

    } // namespace action
    
} // namespace btn
#endif 