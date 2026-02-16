
#include "abv_navigation/SimulatedStateFetcher.h"
#include "common/RosTopicManager.h"


SimulatedStateFetcher::SimulatedStateFetcher()
{
    mAcquired.store(false);  
}

SimulatedStateFetcher::~SimulatedStateFetcher()
{
}

bool SimulatedStateFetcher::init()
{
    RosTopicManager::getInstance()->createSubscriber<abv_msgs::msg::AbvState>("abv/sim/state", 
                                                std::bind(&SimulatedStateFetcher::stateCallback, this, std::placeholders::_1));

    return true; 
}

void SimulatedStateFetcher::stateCallback(abv_msgs::msg::AbvState::SharedPtr aSimState)
{
    AbvState state; 
    state.x = aSimState->position.x;
    state.y = aSimState->position.y; 
    state.theta = aSimState->orientation.z; 

    state.vx = aSimState->velocity.x; 
    state.vy = aSimState->velocity.y; 
    state.omega = aSimState->ang_vel.z; 
    
    // thread safe setting of state
    setState(state); 
    if(!mAcquired.load())
        mAcquired.store(true); 
}

void SimulatedStateFetcher::setState(const AbvState& aState)
{
    std::lock_guard<std::mutex> lock(mStateMutex); 
    mState = aState; 
}

AbvState SimulatedStateFetcher::fetchState()
{
    std::lock_guard<std::mutex> lock(mStateMutex); 
    return mState; 
}