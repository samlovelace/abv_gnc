
#include <thread>

#include "abv_controller/Vehicle.h"
#include "plog/Log.h"
#include "abv_common/RateController.hpp"

#include "abv_controller/PidControlPolicy.h"

Vehicle::Vehicle() : 
    mNavManager(std::make_shared<RosNavigationListener>()),
    mLastInputRecvdAt(std::chrono::steady_clock::now()), mStaleInputThreshold(std::chrono::duration<double>(std::chrono::milliseconds(250))), 
    mPoseError(), mVelError(),
    mArrivalTimerActive(false), 
    mConfig(ConfigurationManager::getInstance()->getControlConfig()), 
    mArrivalTol(mConfig.mPoseArrivalTol),
    mGoalType(GoalType::NUM_TYPES)
{ 
    mController = std::make_unique<PidControlPolicy>();
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
    // bypass control and directly command a sequence of thrusters 
    mThrusterCommander->commandThrusters(mThrusterCmdSequence.get()); 
}

void Vehicle::doDirectionControl()
{
    Eigen::Vector3d controlInput = getControlInput(); 
    Eigen::Vector3d controlInputBodyFrame = convertToBodyFrame(controlInput); 
    mThrusterCommander->command(controlInputBodyFrame);
}

void Vehicle::doPoseControl()
{ 
    ControlContext ctx;
    ctx.currentPose = mNavManager->getCurrentPose();
    ctx.currentVelocity = mNavManager->getCurrentVel();

    ctx.goal = getGoalPose(); 
    ctx.error = ctx.goal - ctx.currentPose; 
    
    // wrap yaw error to [-pi, pi]
    ctx.error[2] = std::atan2(std::sin(ctx.error[2]), std::cos(ctx.error[2]));

    mPoseError.set(ctx.error); 
    Eigen::Vector3d controlInput = mController->computeAction(ctx);

    setControlInput(controlInput);
    doDirectionControl(); 
}

void Vehicle::doVelocityControl()
{
    ControlContext ctx;
    ctx.currentPose = mNavManager->getCurrentPose();
    ctx.currentVelocity = mNavManager->getCurrentVel();
    ctx.goal = getGoalVelocity();

    mVelError.set(ctx.goal - ctx.currentVelocity); 
    Eigen::Vector3d controlInput = mController->computeAction(ctx);

    setControlInput(controlInput); 
    doDirectionControl(); 
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
    LOGV << "Received goal pose: " << aGoalPose; 
    std::lock_guard<std::mutex> lock(mGoalPoseMutex); 
    mGoalPose = aGoalPose;
    mGoalType = GoalType::POSE;
    mJustRecvdNewGoal.set(true); 
}
void Vehicle::setGoalVelocity(Eigen::Vector3d aGoalVel)
{
    LOGV << "Received goal vel: " << aGoalVel; 
    std::lock_guard<std::mutex> lock(mGoalVelocityMutex); 
    mGoalVelocity = aGoalVel; 
    mGoalType = GoalType::VELOCITY;
    mJustRecvdNewGoal.set(true); 
}

void Vehicle::setArrivalTolerance(const Eigen::Vector3d& aTolerance)
{
    std::lock_guard<std::mutex> lock(mGoalToleranceMutex);
    
    for(int i = 0; i < 3; i++)
    {
        // if any field is 0 or negative, use the default configured tolerances,
        // otherwise use the value that was commanded 
        if(0 >= aTolerance[i])
        {
            mArrivalTol[i] = mConfig.mPoseArrivalTol[i];
        }
        else
        {
            mArrivalTol[i] = aTolerance[i]; 
        }
    }

    LOGV << "Using waypoint tolerance of " << mArrivalTol; 
}

void Vehicle::setThrusterCmdSequence(const std::string& aCmd)
{
    LOGV << "Received thruster command sequence " << aCmd; 
    mThrusterCmdSequence.set(aCmd);  
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

    // yaw is element 2 TODO: make this more explicit 
    double yaw = state[2]; 

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
    LOGV << "Received STOP command!"; 
    Eigen::Vector3d zeros = Eigen::Vector3d::Zero();  
    setControlInput(zeros); 
    doThrusterControl();
    mGoalType = GoalType::NUM_TYPES; // so arrival state goes to IDLE 
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
            within_threshold = (abs(error.array()) < mArrivalTol.array()).all();
            break;
        }
    case GoalType::VELOCITY:
        {
            auto error = mVelError.get();
            within_threshold = (abs(error.array()) < mArrivalTol.array()).all();
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
    if (now - mArrivalStart >= std::chrono::seconds((int)mConfig.mArrivalDuration))
    {
        status = Arrival::Status::ARRIVED;
    }
    else
    {
        status = Arrival::Status::RUNNING;
    }

    return status;
}
