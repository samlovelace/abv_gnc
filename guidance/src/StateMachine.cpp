
#include <iostream> 
#include "plog/Log.h"

#include "robot_idl/msg/abv_command.hpp"

#include "common/RateController.hpp"
#include "common/ConfigurationManager.h"
#include "common/RosTopicManager.h"

#include "abv_guidance/StateMachine.h"
#include "abv_guidance/StraightLineGenerator.h"
#include "abv_guidance/FromFileGenerator.h"

StateMachine::StateMachine() : 
    mDone(false), mActiveState(States::STARTUP)
{
    RosTopicManager::getInstance()->createPublisher<robot_idl::msg::AbvCommand>("abv/command"); 
    RosTopicManager::getInstance()->createSubscriber<robot_idl::msg::AbvControllerStatus>("abv/controller_status", 
            std::bind(&StateMachine::controllerStatusCallback, this, std::placeholders::_1)); 
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
        
        case States::WAITING_FOR_EXECUTION: 
            waitForExecution(); 
            break; 

        case States::WAITING_FOR_ARRIVAL:
            waitForArrival(); 
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
        LOGV << "Acquired navigation data!"; 
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
    else if("file" == mCommand.mType)
    {
        auto pathGen = std::make_unique<FromFileGenerator>(); 
        mPathGenerator = std::move(pathGen); 
    }
    else
    {
        LOGW << "Unknown path generator of type: " << mCommand.mType; 
        setActiveState(States::IDLE); 
        return; 
    }

    // init the path generator 
    if(!mPathGenerator->init())
    {
        LOGW << "Failed to initialize path generator..."; 
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
    Waypoint wp = mPathGenerator->getNext(); 

    // convert to idl type and publish 
    robot_idl::msg::AbvCommand cmd; 
    cmd.set__type(wp.mType); 
    
    robot_idl::msg::AbvVec3 vec; 
    vec.set__x(wp.mPose.x()); 
    vec.set__y(wp.mPose.y()); 
    vec.set__yaw(wp.mPose.z()); 

    cmd.set__data(vec); 

    LOGV << "Sending next waypoint..."; 
    RosTopicManager::getInstance()->publishMessage<robot_idl::msg::AbvCommand>("abv/command", cmd); 
    setActiveState(States::WAITING_FOR_EXECUTION); 
}

void StateMachine::waitForExecution()
{
    if(mArrivalStatus.get() == Arrival::Status::RUNNING)
    {
        LOGV << "Controller heard waypoint..."; 
        setActiveState(States::WAITING_FOR_ARRIVAL); 
    }
}

void StateMachine::waitForArrival()
{
    // query controller status for arrival state, once arrived, send next waypoint 
    if(mArrivalStatus.get() == Arrival::Status::ARRIVED)
    {
        // controller arrived on current waypoint, send next
        LOGV << "Controller arrived..."; 
        if(mPathGenerator->hasNext())
        {
            setActiveState(States::SEND_WAYPOINT);
        }
        else
        {
            // arrived at final waypoint, go back to IDLE
            setActiveState(States::IDLE); 
        }
    }
}

void StateMachine::controllerStatusCallback(robot_idl::msg::AbvControllerStatus::SharedPtr aStatus)
{
    mArrivalStatus.set((Arrival::Status)aStatus->arrival); 
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
    case StateMachine::States::WAITING_FOR_EXECUTION: 
        stringToReturn = "WAITING_FOR_EXECUTION"; 
        break; 
    case StateMachine::States::WAITING_FOR_ARRIVAL: 
        stringToReturn = "WAITING_FOR_ARRIVAL"; 
        break; 
    default:
        stringToReturn = "UNKNOWN"; 
        break;
    }

    return stringToReturn; 
}