
#include "autobot_navigation/SimulatedStateFetcher.h"
#include "autobot_common/RosTopicManager.h"


SimulatedStateFetcher::SimulatedStateFetcher(ConsumableBuffer<StampedState>& aBuffer) : IStateFetcher(aBuffer)
{
    mAcquired.store(false);  
}

SimulatedStateFetcher::~SimulatedStateFetcher()
{
}

bool SimulatedStateFetcher::init()
{
    RosTopicManager::getInstance()->createSubscriber<autobot_msgs::msg::State>("abv/sim/state", 
                                                std::bind(&SimulatedStateFetcher::stateCallback, this, std::placeholders::_1));

    return true; 
}

void SimulatedStateFetcher::stateCallback(autobot_msgs::msg::State::SharedPtr aSimState)
{
    State state; 
    state.x = aSimState->position.x;
    state.y = aSimState->position.y; 
    state.theta = aSimState->position.yaw; 

    state.vx = aSimState->velocity.x; 
    state.vy = aSimState->velocity.y; 
    state.omega = aSimState->velocity.yaw;  
    
    // thread safe setting of state
    mBuffer.put(StampedState{state, std::chrono::system_clock::now()});
    if(!mAcquired.load())
        mAcquired.store(true);
}