
#include <iostream> 
#include "plog/Log.h"

#include "common/RateController.hpp"
#include "common/ConfigurationManager.h"
#include "common/RosTopicManager.h"

#include "abv_guidance/StateMachine.h"
#include "abv_guidance/StraightLineGenerator.h"

StateMachine::StateMachine() : 
    mDone(false), mActiveState(States::STARTUP)
{

}

StateMachine::~StateMachine()
{

}

void StateMachine::onCommand(const Command& aCommand)
{
    mCommand = aCommand;
    LOGV << "Recvd command to execute " << mCommand.mType << " path for " << mCommand.mDuration << " seconds!";
    if(States::STARTUP == getActiveState())
    {
        LOGW << "Cannot execute trajectory, navigation data not yet acquired..."; 
        return; 
    }  
    
    setActiveState(States::GENERATE_PATH); 
}

void StateMachine::run()
{
    //auto config = ConfigurationManager::getInstance()->getStateMachineConfig(); 
    RateController rate(10); 

    LOGD << "State Machine starting in " << toString(mActiveState);

    while(!isDone())
    {
        rate.start(); 

        switch (getActiveState())
        {
        case States::STARTUP: 
            startup();  
            break;

        case States::IDLE:
            // do nothing 
            break;

        case States::GENERATE_PATH:
            generatePath(); 
            break; 

        case States::SEND_WAYPOINT:
            sendWaypoint(); 
            break;

        case States::WAITING:
            wait(); 
            break; 

        default:
            break;
        }

        rate.block(); 
    }
}

void StateMachine::startup()
{
    // wait for nav data, once received transition to IDLE.
    if(mNavSource.hasAcquiredStateData())
    {
        setActiveState(States::IDLE);
    }
}

void StateMachine::generatePath()
{
    // get the latest command, instantiate the proper IPathGenerator based on the command
    // and do any startup/init stuff for that IPathGenerator 
    if("line" == mCommand.mType)
    {
        auto pathGen = std::make_unique<StraightLineGenerator>(); 
        mPathGenerator = std::move(pathGen);         
    }
    else
    {
        LOGW << "Unknown path generator of type: " << mCommand.mType; 
        setActiveState(States::IDLE); 
        return; 
    }

    // reset watchdog 
    //mWatchdog.setDuration(mCommand.mDuration); 
    setActiveState(States::SEND_WAYPOINT); 
    //mWatchdog.reset();  
}

void StateMachine::sendWaypoint()
{
    // get next waypoint from current IPathGenerator and send via ROS2 
    if(mPathGenerator->hasNext())
    {
        Waypoint wp = mPathGenerator->getNext(); 

        // convert to idl type and publish 

        // transition to wait state 
        setActiveState(States::WAITING); 
    }
}

void StateMachine::wait()
{
    // query controller status for arrival state, once arrived, send next waypoint 
}

void StateMachine::setActiveState(StateMachine::States aState)
{
    LOGD << "Switching state machine from " << toString(getActiveState()).c_str() 
         << " to " << toString(aState).c_str();

    std::lock_guard<std::mutex> lock(mActiveStateMutex); 
    mActiveState = aState;    
}

std::string StateMachine::toString(StateMachine::States aState)
{
    std::string stringToReturn = ""; 
    switch (aState)
    {
    case StateMachine::States::STARTUP:
        stringToReturn = "STARTUP"; 
        break; 
    case StateMachine::States::IDLE:
        stringToReturn = "IDLE"; 
        break;
    case StateMachine::States::GENERATE_PATH: 
        stringToReturn = "GENERATE_PATH";
        break; 
    case StateMachine::States::SEND_WAYPOINT: 
        stringToReturn = "SEND_WAYPOINT"; 
        break;
    case StateMachine::States::WAITING: 
        stringToReturn = "WAITING"; 
        break; 
    default:
        stringToReturn = "UNKNOWN"; 
        break;
    }

    return stringToReturn; 
}