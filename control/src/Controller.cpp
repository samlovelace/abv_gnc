
#include "abv_controller/Controller.h"
#include "common/ConfigurationManager.h"

Controller::Controller() : mConfig(ConfigurationManager::getInstance()->getControlConfig())
{
    mPrevTime = std::chrono::steady_clock::now(); 
    mPrevPoseError = Eigen::Vector3d::Zero(); 
    mPoseErrorIntegral = Eigen::Vector3d::Zero(); 
}

Controller::~Controller()
{
    
}

Eigen::Vector3d Controller::computeControlInput(Eigen::Vector3d aPoseError)
{
    return PID(aPoseError);
}

Eigen::Vector3d Controller::PID(Eigen::Vector3d aPoseError)
{
    using namespace std::chrono; 
    
    Eigen::Vector3d poseErrorDeriv; 
    Eigen::Vector3d poseErrorIntegral;
    Eigen::Vector3d controlInput = Eigen::Vector3d::Zero(); 

    double dt = duration_cast<milliseconds>(steady_clock::now() - mPrevTime).count() / 1000.0;

    for (int i = 0; i < 3; i++) 
    {
        double error = aPoseError[i];
        double deriv = (error - mPrevPoseError[i]) / dt;
        mPoseErrorIntegral[i] += error * dt;

        controlInput[i] = mConfig.mKp[i] * error +
                          mConfig.mKi[i] * mPoseErrorIntegral[i] +
                          mConfig.mKd[i] * deriv;
    }

    mPrevPoseError = aPoseError; 
    mPrevTime = steady_clock::now();

    return controlInput;
}
