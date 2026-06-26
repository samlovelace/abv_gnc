#ifndef VEHICLE_H
#define VEHICLE_H

#include <eigen3/Eigen/Dense>
#include <mutex> 
#include <memory>

#include "abv_common/ArrivalStatus.hpp"
#include "abv_common/ThreadSafe.hpp"
#include "abv_common/RosNavigationListener.h"

#include "abv_controller/ThrusterCommander.h"
#include "abv_controller/GoalType.hpp"
#include "abv_controller/IControlPolicy.hpp"

class Vehicle
{
public:
    Vehicle();
    ~Vehicle();

    bool init(); 

    void doThrusterControl(); 
    void doDirectionControl(); 
    void doPoseControl(); 
    void doVelocityControl(); 

    void setGoalPose(Eigen::Vector3d aGoalPose);
    void setGoalVelocity(Eigen::Vector3d aGoalVel, bool anIsGlobal = true); 
    void setControlInput(Eigen::Vector3d aControlInput, bool anIsGlobal = true);
    void setThrusterCmdSequence(const std::string& aCmd);

    void setArrivalTolerance(const Eigen::Vector3d& aTolerance);  

    void stop(); 

    Eigen::Vector3d getGoalPose();
    Eigen::Vector3d getGoalVelocity(); 
    Eigen::Vector3d getControlInput(); 

    bool isControlInputStale(); 
    bool hasAcquiredStateData();
    struct ControlStatus
    {
        Eigen::Vector3d mAppliedThrust; 
        Arrival::Status mStatus; 
    };

    ControlStatus getControlStatus(); 

private:
    Eigen::Vector3d mGoalPose; 
    Eigen::Vector3d mGoalVelocity; 
    Eigen::Vector3d mControlInput;

    ControlConfig mConfig; 

    ThreadSafe<Eigen::Vector3d> mPoseError; 
    ThreadSafe<Eigen::Vector3d> mVelError; 
    ThreadSafe<bool> mJustRecvdNewGoal; 
    ThreadSafe<bool> mIsGoalGlobal;

    Eigen::Vector3d mArrivalTol; 
    std::chrono::steady_clock::time_point mArrivalStart;
    bool mArrivalTimerActive;


    std::chrono::steady_clock::time_point mLastInputRecvdAt; 
    std::chrono::duration<double> mStaleInputThreshold;  

    GoalType mGoalType; 

    std::mutex mGoalPoseMutex; 
    std::mutex mControlInputMutex; 
    std::mutex mGoalVelocityMutex;
    std::mutex mGoalToleranceMutex; 

    std::unique_ptr<ThrusterCommander> mThrusterCommander;
    std::shared_ptr<RosNavigationListener> mNavManager;
    std::unique_ptr<IControlPolicy> mController; 

    ThreadSafe<std::string> mThrusterCmdSequence; 

private: 
    Eigen::Vector3d convertToBodyFrame(Eigen::Vector3d aControlInputGlobal); 
    Arrival::Status determineArrivalStatus(); 

};
#endif // VEHICLE_H

