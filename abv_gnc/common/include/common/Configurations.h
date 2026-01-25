#ifndef CONFIGURATIONS_H
#define CONFIGURATIONS_H

#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <plog/Log.h>
struct StateMachineConfig 
{
    int mRate;
};
struct StateTrackerConfig 
{
    std::string mInterface;
    int mRate;
    std::string mServerIp; 
    std::string mLocalIp; 
    int mRigidBodyId; 
};
struct StatePublisherConfig 
{
    std::string mInterface; 
    int mRate; 
};
struct ThrusterConfig 
{
    double uOn;
    double uOff;
    std::string mType; 
    std::vector<int> mGpioPins;
    double mForce; 
    double mMomentArm; 
};
struct ControllerConfig 
{
    Eigen::Vector3d Kp;
    Eigen::Vector3d Ki;
    Eigen::Vector3d Kd;
    ThrusterConfig thrusterConfig;
};
struct VehicleConfig 
{
    std::string Name;
    double Mass;
    double Inertia;

    StateTrackerConfig stateTrackerConfig;
    StatePublisherConfig statePublisherConfig; 
    ControllerConfig controllerConfig;
};
struct Configurations 
{
    StateMachineConfig stateMachineConfig;
    VehicleConfig vehicleConfig;
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