#ifndef THRUSTERCOMMANDER_H
#define THRUSTERCOMMANDER_H

#include <eigen3/Eigen/Dense>
#include "autobot_config/GncConfigurations.h"
#include "autobot_config/GncConfigManager.h"
#include "autobot_thrusters/IThrusterDriver.hpp"
#include "autobot_thrusters/IThrusterMapper.hpp"

#include <mutex>
#include <memory>

class ThrusterCommander
{
public:
    ThrusterCommander(/* args */);
    ~ThrusterCommander();

    void command(Eigen::Vector3d aControlInput);
    void commandThrusters(const std::string& aThrustersCmd);

    Eigen::Vector3d getAppliedThrustVector();
    std::string getThrusterCommand();

protected:
    Eigen::Vector3i convertToThrustVector(Eigen::Vector3d aControlInput);

    ThrusterConfig mConfig;
    std::string mThrusterCommand;
    std::mutex mThrusterCommandMutex;

    double mThrusterForce;
    double mMomentArm;
    Eigen::Vector3d mAppliedThrustVector;

    std::unique_ptr<IThrusterMapper> mThrusterMapper;
    std::unique_ptr<IThrusterDriver> mThrusterDriver;

};

#endif //THRUSTERCOMMANDER_H
