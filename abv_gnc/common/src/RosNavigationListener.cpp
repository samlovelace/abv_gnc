
#include "common/RosNavigationListener.h"
#include "common/RosTopicManager.h"

RosNavigationListener::RosNavigationListener(/* args */) : mAcquiredState(false)
{
    RosTopicManager::getInstance()->createSubscriber<abv_msgs::msg::AbvState>("abv/state", 
                                                                               std::bind(&RosNavigationListener::stateCallback,
                                                                                         this, 
                                                                                         std::placeholders::_1));                                                                                
}

RosNavigationListener::~RosNavigationListener()
{

}

void RosNavigationListener::stateCallback(const abv_msgs::msg::AbvState::SharedPtr aMsg)
{
    Eigen::Matrix<double, 6, 1> state; 
    
    state[0] = aMsg->position.x; 
    state[1] = aMsg->position.y; 
    state[2] = aMsg->position.yaw; 

    state[3] = aMsg->velocity.x; 
    state[4] = aMsg->velocity.y; 
    state[5] = aMsg->velocity.yaw; 

    {
        std::lock_guard<std::mutex> lock(mCurrentStateMutex); 
        mCurrentState = state; 

        if(!mAcquiredState)
        {
            // if this is the first callback invoked, set that we have acquired the state data 
            mAcquiredState = true; 
        }
    }
}

void RosNavigationListener::setState(const Eigen::Matrix<double, 6, 1>& aState)
{
    std::lock_guard<std::mutex> lock(mCurrentStateMutex); 
    mCurrentState = aState; 
}

bool RosNavigationListener::hasAcquiredStateData()
{
    return mAcquiredState; 
}

Eigen::Vector3d RosNavigationListener::getCurrentPose()
{
    std::lock_guard<std::mutex> lock(mCurrentStateMutex); 

    Eigen::Vector3d pose; 
    pose << mCurrentState[0], mCurrentState[1], mCurrentState[2]; 
    
    return pose; 
}

Eigen::Vector3d RosNavigationListener::getCurrentVel()
{
    std::lock_guard<std::mutex> lock(mCurrentStateMutex); 

    Eigen::Vector3d vel; 
    vel << mCurrentState[3], mCurrentState[4], mCurrentState[5]; 

    return vel; 
}

Eigen::Matrix<double, 6, 1> RosNavigationListener::getCurrentState()
{
    std::lock_guard<std::mutex> lock(mCurrentStateMutex); 
    return mCurrentState; 
}