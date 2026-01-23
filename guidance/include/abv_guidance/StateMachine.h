#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <memory>
#include <thread>
#include <mutex>

#include "common/RosNavigationListener.h"

#include "abv_guidance/IPathGenerator.hpp"
#include "abv_guidance/InternalTypes.hpp"
#include "abv_guidance/ICommandSink.hpp"

class StateMachine : public ICommandSink
{
public:
    StateMachine();
    ~StateMachine();

    enum class States
    {
        STARTUP,
        IDLE, 
        GENERATE_PATH, 
        SEND_WAYPOINT, 
        WAITING, 
        NUM_TYPES
    };

    void run(); 

    std::string toString(States aState); 

    /** Getters and Setters **/
    bool isDone() {std::lock_guard<std::mutex> lock(mDoneMutex); return mDone; }
    void stop() {std::lock_guard<std::mutex> lock(mDoneMutex); mDone = true; }
    void setActiveState(States aNewActiveState); 
    States getActiveState() {std::lock_guard<std::mutex> lock(mActiveStateMutex); return mActiveState;}

private:
    bool mDone; 
    std::mutex mDoneMutex; 
    States mActiveState; 
    std::mutex mActiveStateMutex; 

    Command mCommand;

    std::unique_ptr<IPathGenerator> mPathGenerator; 
    RosNavigationListener mNavSource; 

private: 
    void startup(); 
    void generatePath(); 
    void sendWaypoint(); 
    void wait(); 

    void onCommand(const Command& aCommand) override; 

};
#endif // STATEMACHINE_H
