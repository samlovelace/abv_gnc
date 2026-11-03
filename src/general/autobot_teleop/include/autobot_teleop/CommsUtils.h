#ifndef COMMSUTILS_H
#define COMMSUTILS_H

#include "autobot_msgs/msg/controller_command.hpp"
#include <eigen3/Eigen/Dense>

namespace CommsUtils
{
    static autobot_msgs::msg::ControllerCommand commandMsgToIdl(const Eigen::VectorXd& aData)
    {
        autobot_msgs::msg::ControllerCommand cmd;
        autobot_msgs::msg::Vec3 vec; 
        vec.set__x(aData[0]); 
        vec.set__y(aData[1]); 
        vec.set__yaw(aData[2]);  
        cmd.set__type("direction"); 
        cmd.set__data(vec); 

        return cmd; 
    }
};

#endif