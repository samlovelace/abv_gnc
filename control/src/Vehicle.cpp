
#include <thread>

#include "abv_controller/Vehicle.h"
#include "plog/Log.h"
#include "common/RateController.hpp"


Vehicle::Vehicle() : 
    mNavManager(std::make_shared<RosNavigationListener>()), mController(std::make_unique<Controller>()),
    mLastInputRecvdAt(std::chrono::steady_clock::now()), mStaleInputThreshold(std::chrono::duration<double>(std::chrono::milliseconds(500)))
{
}

Vehicle::~Vehicle()
{
}

bool Vehicle::init()
{
    mThrusterCommander = std::make_unique<ThrusterCommander>();
    //mThrusterCommander->init(); 
}

void Vehicle::doThrusterControl()
{
    Eigen::Vector3d controlInput = getControlInput(); 
    Eigen::Vector3d controlInputBodyFrame = convertToBodyFrame(controlInput); 
    mThrusterCommander->commandThrusters(controlInputBodyFrame);
}

void Vehicle::doPoseControl()
{ 
    Eigen::Vector3d currentPose = mNavManager->getCurrentPose();

    Eigen::Vector3d poseError = getGoalPose() - currentPose; 
    Eigen::Vector3d controlInput = mController->computeControlInput(poseError); 

    setControlInput(controlInput);
    doThrusterControl(); 
}

void Vehicle::doVelocityControl()
{
    Eigen::Vector3d currentVel = mNavManager->getCurrentVel();

    Eigen::Vector3d velError = getGoalVelocity() - currentVel; 
    Eigen::Vector3d controlInput = mController->computeControlInput(velError); 

    setControlInput(controlInput); 
    doThrusterControl(); 
}

void Vehicle::setControlInput(Eigen::Vector3d aControlInput)
{
    // pls dont banish me for using a single mutex on two resources 
    std::lock_guard<std::mutex> lock(mControlInputMutex); 
    mControlInput = aControlInput;
    mLastInputRecvdAt = std::chrono::steady_clock::now(); 
}

void Vehicle::setGoalPose(Eigen::Vector3d aGoalPose) 
{
    std::lock_guard<std::mutex> lock(mGoalPoseMutex); 
    mGoalPose = aGoalPose;
}
void Vehicle::setGoalVelocity(Eigen::Vector3d aGoalVel)
{
    std::lock_guard<std::mutex> lock(mGoalVelocityMutex); 
    mGoalVelocity = aGoalVel; 
}

Eigen::Vector3d Vehicle::getGoalPose() 
{
    std::lock_guard<std::mutex> lock(mGoalPoseMutex); 
    return mGoalPose; 
}

Eigen::Vector3d Vehicle::getGoalVelocity() 
{
    std::lock_guard<std::mutex> lock(mGoalVelocityMutex);
    return mGoalVelocity; 
}

Eigen::Vector3d Vehicle::getControlInput() 
{
    std::lock_guard<std::mutex> lock(mControlInputMutex); 
    return mControlInput; 
}

bool Vehicle::isControlInputStale() 
{
    return std::chrono::steady_clock::now() - mLastInputRecvdAt > mStaleInputThreshold ? true : false;
}

Eigen::Vector3d Vehicle::convertToBodyFrame(Eigen::Vector3d aControlInputGlobal)
{   
    auto state = mNavManager->getCurrentState();

    // yaw is element 6 
    double yaw = state[6]; 

    // invert rotation relative to global 
    Eigen::Matrix3d Rz;
    Rz << cos(yaw), sin(yaw), 0,
         -sin(yaw), cos(yaw), 0,
             0,        0,     1;

    // Transform the vector into the new frame
    return Rz * aControlInputGlobal;
}

bool Vehicle::hasAcquiredStateData()
{
    return mNavManager->hasAcquiredStateData(); 
}

void Vehicle::stop()
{
    Eigen::Vector3d zeros = Eigen::Vector3d::Zero();  
    setControlInput(zeros); 
    doThrusterControl(); 
}

Eigen::Vector3d Vehicle::getControlStatus()
{
    // TODO: expand this to include more controller status related stuff
    // For now, just getting the theoretical thruster vector 
    // TODO: logic to determine some sort of Arrival notion, send that on controller status 
    // Status: IDLE, RUNNING, ARRIVED

    return mThrusterCommander->getAppliedThrustVector(); 
}