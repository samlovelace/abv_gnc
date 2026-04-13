
#include "abv_controller/StateMachine.h"
#include "abv_common/RateController.hpp"
#include "abv_common/ConfigurationManager.h"
#include "abv_common/RosTopicManager.h"
#include <iostream> 
#include "plog/Log.h"

#include "abv_msgs/msg/abv_controller_status.hpp"

StateMachine::StateMachine(std::shared_ptr<Vehicle> abv) : 
    mDone(false), mActiveState(States::STARTUP), mVehicle(abv)
{
    RosTopicManager::getInstance()->createPublisher<abv_msgs::msg::AbvControllerStatus>("abv/controller/status"); 
}

StateMachine::~StateMachine()
{
    if(mControlStatusPublishThread.joinable())
    {
        mControlStatusPublishThread.join(); 
    }
}

void StateMachine::run()
{
    int rateConfig = ConfigurationManager::getInstance()->getControlConfig().mStateMachineRate; 
    RateController rate(rateConfig); 

    mControlStatusPublishThread = std::thread(&StateMachine::controlStatusPublishLoop, this);

    LOGD << "State Machine starting in " << toString(mActiveState);

    while(!isDone())
    {
        rate.start(); 

        switch (getActiveState())
        {
        case States::STARTUP: 
            
            setActiveState(States::IDLE); 
            
            // if(mVehicle->hasAcquiredStateData())
            // {
            //     setActiveState(States::IDLE); 
            //     break; 
            // }

        case States::IDLE:
            // do nothing 
            break;

        case States::THRUSTER_CONTROL: 
            mVehicle->doThrusterControl(); 
            break; 
        case States::DIRECTION_CONTROL:

            if(mVehicle->isControlInputStale())
            {
                mVehicle->stop(); 
                setActiveState(States::IDLE);
                break; 
            }    

            // if here, control input not stale, apply it 
            mVehicle->doDirectionControl(); 
            break;

        case States::POSE_CONTROL: 
        
            mVehicle->doPoseControl(); 
            break;

        case States::VELOCITY_CONTROL: 

            mVehicle->doVelocityControl(); 
            break; 

        default:
            break;
        }

        rate.block(); 
    }
}

void StateMachine::setActiveState(StateMachine::States aState)
{
    LOGD << "Switching state machine from " << toString(getActiveState()).c_str() << " to " 
                                                    << toString(aState).c_str();

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
    case StateMachine::States::THRUSTER_CONTROL: 
        stringToReturn = "THRUSTER_CONTROL";
        break; 
    case StateMachine::States::POSE_CONTROL: 
        stringToReturn = "POSE_CONTROL"; 
        break;
    case StateMachine::States::VELOCITY_CONTROL: 
        stringToReturn = "VELOCITY_CONTROL"; 
        break; 
    case StateMachine::States::DIRECTION_CONTROL:
        stringToReturn = "DIRECTION_CONTROL"; 
        break; 
    default:
        stringToReturn = "UNKNOWN"; 
        break;
    }

    return stringToReturn; 
}

void StateMachine::controlStatusPublishLoop()
{
    // publish control status at same rate as StateMachine loop 
    int rateConfig = ConfigurationManager::getInstance()->getControlConfig().mStateMachineRate; 
    RateController rate(rateConfig); 

    abv_msgs::msg::AbvControllerStatus statusToSend;
    abv_msgs::msg::AbvNodeStatus nodeStatus; 
    nodeStatus.set__node_name("controller");
 
    LOGV << "Starting control status publish loop"; 

    while(!isDone())
    {
        rate.start(); 

        nodeStatus.set__node_state(toString(getActiveState())); 
        statusToSend.set__status(nodeStatus); 

        Vehicle::ControlStatus status = mVehicle->getControlStatus(); 
        statusToSend.set__fx(status.mAppliedThrust.x()); 
        statusToSend.set__fy(status.mAppliedThrust.y()); 
        statusToSend.set__tz(status.mAppliedThrust.z());
        
        statusToSend.set__arrival((uint8_t)status.mStatus); 

        RosTopicManager::getInstance()->publishMessage<abv_msgs::msg::AbvControllerStatus>("abv/controller/status", statusToSend); 

        rate.block(); 
    }
}