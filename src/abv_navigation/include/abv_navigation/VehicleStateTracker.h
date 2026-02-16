#ifndef VEHICLESTATETRACKER_H
#define VEHICLESTATETRACKER_H

#include <memory>
#include <thread> 

#include "abv_navigation/IStateFetcher.h"
#include "common/Configurations.h"
#include "common/ConfigurationManager.h"
#include "RosStatePublisher.h"
#include "ExtendedKalmanFilter.h"

#include "abv_msgs/msg/abv_controller_status.hpp"

class VehicleStateTracker
{
public:
    VehicleStateTracker();
    ~VehicleStateTracker();

    enum class FetcherType
    {
        SIMULATED, 
        OPTITRACK, 
        VICON, 
        NUM_TYPES
    };

    void stateTrackerLoop(); 

    bool doStateTracking() {std::lock_guard<std::mutex> lock(mStateTrackingMutex); return mDoStateTracking; }
    void setStateTracking(bool aFlag) {std::lock_guard<std::mutex> lock(mStateTrackingMutex); mDoStateTracking = aFlag; }

private:
    // polymorphic state fetcher interface so we arent tied to optiTrack
    std::shared_ptr<IStateFetcher> mStateFetcher; // state fetcher interface class 
    RosStatePublisher mStatePublisher; 
    NavigationConfig mConfig; 

    // TODO: add subscription to ControllerStatus, pass latest input to EKF
    ExtendedKalmanFilter mEKF; 

    bool mDoStateTracking;

    std::mutex mStateTrackingMutex; 
    std::thread mStateTrackingThread;  

    VehicleStateTracker::FetcherType toEnum(std::string aTrackerType);

    std::mutex mControlInputMutex; 
    Eigen::Vector3d mLatestControlInput; 
    void controllerStatusCallback(abv_msgs::msg::AbvControllerStatus::ConstSharedPtr aMsg);
    
};
#endif // VEHICLESTATETRACKER_H