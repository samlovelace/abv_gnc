#ifndef THRUSTERCOMMANDER_H
#define THRUSTERCOMMANDER_H

#include <eigen3/Eigen/Dense>
#include "abv_common/Configurations.h"
#include "abv_common/ConfigurationManager.h"
#include "abv_controller/IThrusterDriver.hpp"
#include "abv_controller/IThrusterMapper.hpp"

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

protected:
    Eigen::Vector3i convertToThrustVector(Eigen::Vector3d aControlInput);

    ControlConfig mConfig;
    std::string mThrusterCommand;
    std::mutex mThrusterCommandMutex;

    double mThrusterForce;
    double mMomentArm;
    Eigen::Vector3d mAppliedThrustVector;

    std::unique_ptr<IThrusterMapper> mThrusterMapper;
    std::unique_ptr<IThrusterDriver> mThrusterDriver;

};

#endif //THRUSTERCOMMANDER_H
