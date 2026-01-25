#ifndef THRUSTERCOMMANDER_H
#define THRUSTERCOMMANDER_H

#include <eigen3/Eigen/Dense>
#include "common/Configurations.h"
#include "common/ConfigurationManager.h"
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

    ControlConfig mConfig;  
    std::string mThrusterCommand; 
    std::mutex mThrusterCommandMutex;
    Eigen::Matrix<int, 3, 27> mMatrixOfThrustDirCombinations; 

    double mThrusterForce; 
    double mMomentArm; 
    Eigen::Vector3d mAppliedThrustVector; 

    std::unique_ptr<IThrusterDriver> mThrusterDriver; 

};

#endif //THRUSTERCOMMANDER_H
