#ifndef UTILS_HPP
#define UTILS_HPP

#include "autobot_msgs/msg/autobot_vec3.hpp"
#include <Eigen/Dense>

namespace utils
{
    template<typename T>
    T clamp(const T& value, const T& min, const T& max)
    {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }
    
    autobot_msgs::msg::Vec3 convertToVec3(const Eigen::Vector3d& vec)
    {
        autobot_msgs::msg::Vec3 abvVec;
        abvVec.x = vec[0];
        abvVec.y = vec[1];
        abvVec.yaw = vec[2];
        return abvVec;
    }

    Eigen::Vector3d convertToEigenVec3(const autobot_msgs::msg::Vec3& abvVec)
    {
        return Eigen::Vector3d(abvVec.x, abvVec.y, abvVec.yaw);
    }
}

#endif // UTILS_HPP
