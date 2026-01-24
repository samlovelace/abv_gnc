#ifndef VEHICLE_H
#define VEHICLE_H

#include <eigen3/Eigen/Dense>
#include <mutex> 
#include <memory>

#include "common/ArrivalStatus.hpp"
#include "common/ThreadSafe.hpp"
#include "common/RosNavigationListener.h"

#include "abv_controller/ThrusterCommander.h"
#include "abv_controller/Controller.h"

class Vehicle
{
public:
    Vehicle();
    ~Vehicle();

    bool init(); 

    void doThrusterControl(); 
    void doPoseControl(); 
    void doVelocityControl(); 

    void setGoalPose(Eigen::Vector3d aGoalPose);
    void setGoalVelocity(Eigen::Vector3d aGoalVel); 
    void setControlInput(Eigen::Vector3d aControlInput); 

    void stop(); 

    Eigen::Vector3d getGoalPose();
    Eigen::Vector3d getGoalVelocity(); 
    Eigen::Vector3d getControlInput(); 

    bool isControlInputStale(); 
    bool hasAcquiredStateData(); 

    enum class GoalType
    {
        THRUSTER, 
        POSE, 
        VELOCITY,
        NUM_TYPES
    };
    
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

    ThreadSafe<Eigen::Vector3d> mPoseError; 
    ThreadSafe<Eigen::Vector3d> mVelError; 

    Eigen::Vector3d mPoseThresh; 
    Eigen::Vector3d mVelThresh; 
    std::chrono::steady_clock::time_point mArrivalStart;
    bool mArrivalTimerActive;


    std::chrono::steady_clock::time_point mLastInputRecvdAt; 
    std::chrono::duration<double> mStaleInputThreshold;  

    GoalType mGoalType; 

    std::mutex mGoalPoseMutex; 
    std::mutex mControlInputMutex; 
    std::mutex mGoalVelocityMutex;

    std::unique_ptr<ThrusterCommander> mThrusterCommander;
    std::shared_ptr<RosNavigationListener> mNavManager;
    std::unique_ptr<Controller> mController; 

private: 
    Eigen::Vector3d convertToBodyFrame(Eigen::Vector3d aControlInputGlobal); 
    Arrival::Status determineArrivalStatus(); 

};
#endif // VEHICLE_H

