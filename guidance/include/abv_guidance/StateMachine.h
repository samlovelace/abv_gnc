#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <memory>
#include <thread>
#include <mutex>

#include "robot_idl/msg/abv_controller_status.hpp"

#include "common/ArrivalStatus.hpp"
#include "common/RosNavigationListener.h"
#include "common/ThreadSafe.hpp"

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
        WAITING_FOR_EXECUTION,
        WAITING_FOR_ARRIVAL, 
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

    ThreadSafe<Arrival::Status> mArrivalStatus; 

private: 
    void startup(); 
    void generatePath(); 
    void sendWaypoint(); 
    void waitForExecution();
    void waitForArrival();  

    void onCommand(const Command& aCommand) override; 
    void controllerStatusCallback(robot_idl::msg::AbvControllerStatus::SharedPtr aStatus); 

};
#endif // STATEMACHINE_H
