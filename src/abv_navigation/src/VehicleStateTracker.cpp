
#include "abv_navigation/VehicleStateTracker.h"
#include "abv_navigation/OptitrackStateFetcher_LibMocap.h"
#include "abv_navigation/SimulatedStateFetcher.h"
#include "common/RateController.hpp"
#include "common/DataLogger.h"
#include "common/RosTopicManager.h"
#include "plog/Log.h"
#include <thread>

using OptitrackStateFetcher = OptitrackStateFetcher_LibMocap; 

VehicleStateTracker::VehicleStateTracker() : 
    mConfig(ConfigurationManager::getInstance()->getNavigationConfig())
{
    FetcherType typeToMake = toEnum(mConfig.mInterface); 

    switch (typeToMake)
    {
        case FetcherType::SIMULATED:
            
            mStateFetcher = std::make_shared<SimulatedStateFetcher>(); 
            LOGD << "Configuring ABV to use Simulated state data"; 
            
            break;
        case FetcherType::OPTITRACK: 
            
            mStateFetcher = std::make_shared<OptitrackStateFetcher>(mConfig.mServerIp, mConfig.mLocalIp, mConfig.mRigidBodyName); 
            LOGD << "Configuring ABV to use OptiTrack for state data"; 
            
            break;
        default:
            break;
    }

    setStateTracking(false); 
    mStateTrackingThread = std::thread(&VehicleStateTracker::stateTrackerLoop, this); 

    // setup subscriber to get control status 
    RosTopicManager::getInstance()->createSubscriber<abv_msgs::msg::AbvControllerStatus>("abv/controller_status", 
                                                                                          std::bind(&VehicleStateTracker::controllerStatusCallback, 
                                                                                          this, 
                                                                                          std::placeholders::_1)); 
}

VehicleStateTracker::~VehicleStateTracker()
{
    setStateTracking(false); 

    if(mStateTrackingThread.joinable())
    {
        LOGD << "Joining state tracking thread"; 
        mStateTrackingThread.join(); 
    }

}

VehicleStateTracker::FetcherType VehicleStateTracker::toEnum(std::string aTrackerType)
{
    VehicleStateTracker::FetcherType enumToReturn; 

    if("Simulated" == aTrackerType || "simulated" == aTrackerType)
    {
        enumToReturn = VehicleStateTracker::FetcherType::SIMULATED; 
    }
    else if ("Optitrack" == aTrackerType || "optitrack" == aTrackerType || "optiTrack" == aTrackerType || "OptiTrack" == aTrackerType)
    {
        enumToReturn = VehicleStateTracker::FetcherType::OPTITRACK; 
    }
    else
    {
        LOGE << "Unsupported state fetcher type: " << aTrackerType; 
    }

    return enumToReturn; 
}

void VehicleStateTracker::stateTrackerLoop()
{  
    RateController rate(mConfig.mRate); 
    
    if(!mStateFetcher->init())
    {
        LOGE << "Could not initialize state fetcher of type: " << mConfig.mInterface; 
        return; 
    }
    
    // wait until state is acquired 
    while(!mStateFetcher->isStateAcquired())
    {
        LOGD << "Waiting for state data from " << mStateFetcher->type();
        sleep(1); 
    }

    // init stuff
    auto logId = DataLogger::get().createLog("abv_state_data"); 
    LOGD << "Starting state tracking thread";
    setStateTracking(true); 
    AbvState stateEstimate; 

    while(doStateTracking())
    {
        rate.start(); 

        AbvState state = mStateFetcher->fetchState();
        mEKF.step(state, rate.getDeltaTime(), stateEstimate); 

        mStatePublisher.publish(stateEstimate);  
        DataLogger::get().write(logId, toVector(stateEstimate)); 

        rate.block(); 
    }
}

void VehicleStateTracker::controllerStatusCallback(abv_msgs::msg::AbvControllerStatus::ConstSharedPtr aMsg)
{
    Eigen::Vector3d appliedThrust; 
    appliedThrust << aMsg->fx, aMsg->fy, aMsg->tz; 
    mEKF.setLatestInput(appliedThrust); 
}