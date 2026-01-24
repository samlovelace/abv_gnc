
#include <thread>

#include "abv_controller/Vehicle.h"
#include "plog/Log.h"
#include "common/RateController.hpp"


Vehicle::Vehicle() : 
    mNavManager(std::make_shared<RosNavigationListener>()), mController(std::make_unique<Controller>()),
    mLastInputRecvdAt(std::chrono::steady_clock::now()), mStaleInputThreshold(std::chrono::duration<double>(std::chrono::milliseconds(500))), 
    mPoseError(), mVelError(), mPoseThresh(0.01, 0.01, 0.05), mVelThresh(0.1, 0.1, 0.1), 
    mArrivalTimerActive(false)
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

    mPoseError.set(getGoalPose() - currentPose); 
    Eigen::Vector3d controlInput = mController->computeControlInput(mPoseError.get()); 

    setControlInput(controlInput);
    doThrusterControl(); 
}

void Vehicle::doVelocityControl()
{
    Eigen::Vector3d currentVel = mNavManager->getCurrentVel();

    mVelError.set(getGoalVelocity() - currentVel); 
    Eigen::Vector3d controlInput = mController->computeControlInput(mVelError.get()); 

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
    LOGV << "Recvd goal pose: " << aGoalPose; 
    std::lock_guard<std::mutex> lock(mGoalPoseMutex); 
    mGoalPose = aGoalPose;
    mGoalType = GoalType::POSE;
    mJustRecvdNewGoal.set(true); 
}
void Vehicle::setGoalVelocity(Eigen::Vector3d aGoalVel)
{
    LOGV << "Recd goal vel: " << aGoalVel; 
    std::lock_guard<std::mutex> lock(mGoalVelocityMutex); 
    mGoalVelocity = aGoalVel; 
    mGoalType = GoalType::VELOCITY;
    mJustRecvdNewGoal.set(true); 
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

Vehicle::ControlStatus Vehicle::getControlStatus()
{
    // TODO: expand this to include more controller status related stuff

    ControlStatus cs; 
    cs.mStatus = determineArrivalStatus(); 
    cs.mAppliedThrust = mThrusterCommander->getAppliedThrustVector();     
    
    return cs;  
}

Arrival::Status Vehicle::determineArrivalStatus()
{
    using clock = std::chrono::steady_clock;

    Arrival::Status status;
    status = Arrival::Status::ARRIVED;

    bool within_threshold = false;

    switch (mGoalType)
    {
    case GoalType::POSE:
        {
            auto error = mPoseError.get();
            within_threshold = (abs(error.array()) < mPoseThresh.array()).all();
            break;
        }
    case GoalType::VELOCITY:
        {
            auto error = mVelError.get();
            within_threshold = (abs(error.array()) < mVelThresh.array()).all();
            break;
        }
    case GoalType::THRUSTER:
        {
            status = Arrival::Status::RUNNING;
            return status;
        }
    default:
        status = Arrival::Status::IDLE;
        return status;
    }

    // --- Arrival timing logic ---
    const auto now = clock::now();
    const auto required_duration = std::chrono::milliseconds(5000); // TODO: make config 

    if (!within_threshold || mJustRecvdNewGoal.get())
    {
        // Reset timer
        mArrivalTimerActive = false;
        mJustRecvdNewGoal.set(false); 
        status = Arrival::Status::RUNNING;
        return status;
    }

    // We are within threshold
    if (!mArrivalTimerActive)
    {
        // Start the timer
        mArrivalTimerActive = true;
        mArrivalStart = now;
        status = Arrival::Status::RUNNING;
        return status;
    }

    // Timer is active — check if enough time has passed
    if (now - mArrivalStart >= required_duration)
    {
        status = Arrival::Status::ARRIVED;
    }
    else
    {
        status = Arrival::Status::RUNNING;
    }

    return status;
}
