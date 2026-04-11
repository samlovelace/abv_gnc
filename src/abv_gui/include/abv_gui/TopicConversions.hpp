#ifndef TOPICCONVERSIONS_HPP
#define TOPICCONVERSIONS_HPP


// incoming types 
#include "abv_msgs/msg/abv_state.hpp"
#include "abv_msgs/msg/abv_controller_status.hpp"

// outgoing types
#include <QVector>

namespace conversions
{
    inline QVector<double> 
    navigationPositionConvertor(const abv_msgs::msg::AbvState& msg)
    {
        return QVector<double>{
            msg.position.x,
            msg.position.y,
            msg.position.yaw
        }; 
    }

    inline QVector<double> 
    navigationVelocityConvertor(const abv_msgs::msg::AbvState& msg)
    {
        return QVector<double> {
            msg.velocity.x, 
            msg.velocity.y, 
            msg.velocity.yaw
        };
    }

    inline QVector<double> 
    controllerStatusConvertor(const abv_msgs::msg::AbvControllerStatus& msg)
    {
        return QVector<double> {
            msg.fx, 
            msg.fy, 
            msg.tz
        };
    }

} // namespace topic

#endif