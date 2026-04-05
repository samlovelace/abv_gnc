#ifndef THRUSTERCOMMANDER_H
#define THRUSTERCOMMANDER_H

#include <eigen3/Eigen/Dense>
#include "abv_common/Configurations.h"
#include "abv_common/ConfigurationManager.h"
#include "abv_controller/IThrusterDriver.hpp"

#include <mutex> 
#include <memory>

class ThrusterCommander
{
public:
    ThrusterCommander(/* args */);
    ~ThrusterCommander();

    void commandThrusters(Eigen::Vector3d aControlInput); 

    Eigen::Vector3d getAppliedThrustVector(); 

protected:
    Eigen::Vector3i convertToThrustVector(Eigen::Vector3d aControlInput); 
    void determineThrusterCommand(Eigen::Vector3i aThrustDirVec);
    void allocate(const Eigen::Vector3i& aThrustDir);

    ControlConfig mConfig;  
    std::string mThrusterCommand; 
    std::mutex mThrusterCommandMutex;
    Eigen::Matrix<int, 3, 27> mMatrixOfThrustDirCombinations; 
    static constexpr double THRESHOLD = 0.2;

    Eigen::Matrix<double, 3, 8> mB;
    Eigen::Matrix<double, 8, 3> mBpinv;
    Eigen::Array<bool, 8, 1>    mThrusterCommands;

    double mThrusterForce; 
    double mMomentArm; 
    Eigen::Vector3d mAppliedThrustVector; 

    std::unique_ptr<IThrusterDriver> mThrusterDriver; 

};

#endif //THRUSTERCOMMANDER_H
