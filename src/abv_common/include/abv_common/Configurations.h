#ifndef CONFIGURATIONS_H
#define CONFIGURATIONS_H

#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <plog/Log.h>

struct VehicleConfig 
{
    std::string Name;
    double Mass;
    double Inertia;
};

struct GuidanceConfig
{
    int mStateMachineRate;  
};

struct NavigationConfig
{
    std::string mInterface;
    int mRate;
    std::string mServerIp; 
    std::string mLocalIp; 
    std::string mRigidBodyName;
};

struct ControlConfig
{
    int mStateMachineRate; 

    // PID controller gains 
    Eigen::Vector3d mKp;
    Eigen::Vector3d mKi;
    Eigen::Vector3d mKd;

    // Controller arrival
    Eigen::Vector3d mPoseArrivalTol;
    double mArrivalDuration; 

    // ThrusterCommander & ThrusterDriver configs
    std::string mThrusterDriverType; 
    std::vector<int> mGpioPins;
    double mSchmittTriggerOn;
    double mSchmittTriggerOff;
    double mForce; 
    double mMomentArm; 
};

namespace ConfigUtils
{
    static Eigen::Vector3d parseVector3d(const YAML::Node& node) 
    {
        Eigen::Vector3d vec;

        if (node && node.IsSequence() && node.size() == 3) {
            vec[0] = node[0].as<double>();
            vec[1] = node[1].as<double>();
            vec[2] = node[2].as<double>();
        } 
        else 
        {
            throw std::runtime_error("Invalid Vector3d format in YAML.");
        }

        return vec;
    }

    static std::vector<int> parseIntVector(const YAML::Node& node)
    {
        if (!node || !node.IsSequence()) 
        {
            throw std::runtime_error("Invalid int vector format in YAML.");
        }

        std::vector<int> vec;
        vec.reserve(node.size());  // Optional, improves efficiency

        for (const auto& element : node) 
        {
            vec.push_back(element.as<int>());
        }

        return vec;
    }
}

#endif // CONFIGURATIONS_H