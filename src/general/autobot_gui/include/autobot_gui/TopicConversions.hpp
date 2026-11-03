#ifndef TOPICCONVERSIONS_HPP
#define TOPICCONVERSIONS_HPP


// incoming types
#include "autobot_msgs/msg/state.hpp"
#include "autobot_msgs/msg/controller_status.hpp"
#include "autobot_msgs/msg/controller_command.hpp"
#include "autobot_msgs/msg/thruster_status.hpp"
#include "autobot_msgs/msg/path.hpp"

// outgoing types
#include <QVector>
#include <QPointF>

namespace conversions
{
    inline QVector<double> 
    navigationPositionConvertor(const autobot_msgs::msg::State& msg)
    {
        return QVector<double>{
            msg.position.x,
            msg.position.y,
            msg.position.yaw
        }; 
    }

    inline QVector<double>
    navigationStateConvertor(const autobot_msgs::msg::State& msg)
    {
        return QVector<double>{
            msg.position.x,
            msg.position.y,
            msg.position.yaw,
            msg.valid ? 1.0 : 0.0
        };
    }

    inline QVector<double>
    navigationVelocityConvertor(const autobot_msgs::msg::State& msg)
    {
        return QVector<double> {
            msg.velocity.x, 
            msg.velocity.y, 
            msg.velocity.yaw
        };
    }

    inline QVector<double>
    thrusterStatusConvertor(const autobot_msgs::msg::ThrusterStatus& msg)
    {
        return QVector<double> {
            msg.fx,
            msg.fy,
            msg.tz
        };
    }

    inline QVector<QPointF>
    pathConvertor(const autobot_msgs::msg::Path& msg)
    {
        QVector<QPointF> path;
        path.reserve(static_cast<int>(msg.waypoints.size()));
        for (const auto& wp : msg.waypoints)
        {
            path.push_back(QPointF(wp.x, wp.y));
        }
        return path;
    }

} // namespace topic
#endif