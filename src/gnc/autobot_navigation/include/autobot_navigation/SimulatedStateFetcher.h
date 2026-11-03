#ifndef SIMULATEDSTATEFETCHER_H
#define SIMULATEDSTATEFETCHER_H 

#include "autobot_navigation/IStateFetcher.h"
#include "autobot_msgs/msg/state.hpp"

class SimulatedStateFetcher : public IStateFetcher
{
public:
    SimulatedStateFetcher(ConsumableBuffer<StampedState>& aBuffer);
    ~SimulatedStateFetcher() override; 

    bool init() override; 
    std::string type() {return "Simulated"; }

private:

    void stateCallback(autobot_msgs::msg::State::SharedPtr aSimState); 
    void setState(const State& aState); 

private: 
    
    std::mutex mStateMutex;
};

#endif // SIMULATEDSTATEFETCHER_H

