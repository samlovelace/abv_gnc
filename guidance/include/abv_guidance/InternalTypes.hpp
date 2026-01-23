#ifndef INTERNALTYPES_HPP
#define INTERNALTYPES_HPP

#include <string> 
#include <Eigen/Dense>

struct Command
{
    std::string mType; 
    double mDuration; 
};

struct Waypoint
{
    std::string mType; 

    Eigen::Vector3d mPose; 
    Eigen::Vector3d mVel; 
};

#endif 