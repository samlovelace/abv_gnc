#ifndef SIMULATEDSTATEFETCHER_H
#define SIMULATEDSTATEFETCHER_H 

#include "abv_navigation/IStateFetcher.h"
#include "abv_msgs/msg/abv_state.hpp"

class SimulatedStateFetcher : public IStateFetcher
{
public:
    SimulatedStateFetcher(/* args */);
    ~SimulatedStateFetcher() override; 

    bool init() override; 
    AbvState fetchState() override; 
    std::string type() {return "Simulated"; }

private:

    void stateCallback(abv_msgs::msg::AbvState::SharedPtr aSimState); 
    void setState(const AbvState& aState); 

private: 
    
    std::mutex mStateMutex;
};

#endif // SIMULATEDSTATEFETCHER_H

