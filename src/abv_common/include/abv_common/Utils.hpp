#ifndef UTILS_HPP
#define UTILS_HPP

#include "abv_msgs/msg/abv_vec3.hpp"
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
    
    abv_msgs::msg::AbvVec3 convertToAbvVec3(const Eigen::Vector3d& vec)
    {
        abv_msgs::msg::AbvVec3 abvVec;
        abvVec.x = vec[0];
        abvVec.y = vec[1];
        abvVec.yaw = vec[2];
        return abvVec;
    }

    Eigen::Vector3d convertToEigenVec3(const abv_msgs::msg::AbvVec3& abvVec)
    {
        return Eigen::Vector3d(abvVec.x, abvVec.y, abvVec.yaw);
    }
}

#endif // UTILS_HPP
