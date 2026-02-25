
#include "abv_navigation/SimulatedStateFetcher.h"
#include "common/RosTopicManager.h"


SimulatedStateFetcher::SimulatedStateFetcher(ConsumableBuffer<AbvState>& aBuffer) : IStateFetcher(aBuffer)
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
    mBuffer.put(state);  
    if(!mAcquired.load())
        mAcquired.store(true); 
}