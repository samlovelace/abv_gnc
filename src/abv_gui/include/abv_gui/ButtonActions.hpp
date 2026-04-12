#ifndef BUTTONACTIONS_HPP
#define BUTTONACTIONS_HPP

#include "abv_common/RosTopicManager.h"
#include "abv_msgs/msg/abv_controller_command.hpp"

namespace btn
{
    namespace action
    {
        inline void stop()
        {
            abv_msgs::msg::AbvControllerCommand cmd; 
            cmd.set__type("STOP"); 

            RosTopicManager::getInstance()->publishMessage("abv/controller/command", cmd); 
        }

    } // namespace action
    
} // namespace btn
#endif 