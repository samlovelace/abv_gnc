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

    Waypoint(double x, double y, double yaw, const std::string& aType) 
        : mType(aType), mPose(Eigen::Vector3d(x, y, yaw))
    {;}
    
};

#endif 