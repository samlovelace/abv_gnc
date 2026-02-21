
#include "abv_controller/CommandHandler.h"
#include "plog/Log.h"


CommandHandler::CommandHandler(std::shared_ptr<StateMachine> msm, std::shared_ptr<Vehicle> abv) : 
    mStateMachine(msm), mVehicle(abv)
{
    auto topicManager = RosTopicManager::getInstance(); 
    topicManager->createSubscriber<abv_msgs::msg::AbvCommand>("abv/command", 
                                    std::bind(&CommandHandler::commandCallback, this, std::placeholders::_1)); 

    topicManager->spinNode(); 

    while (!topicManager->isROSInitialized())
    {
    }
    
    LOGD << "ROS Comms Initialized";
}

CommandHandler::~CommandHandler()
{
    
}

void CommandHandler::commandCallback(abv_msgs::msg::AbvCommand::SharedPtr aCmdMsg)
{   
    if(CommandType::STOP == toEnum(aCmdMsg->type))
    {
        mVehicle->stop(); 
        setNewActiveState(StateMachine::States::IDLE); 
    }

    if(CommandType::THRUSTER == toEnum(aCmdMsg->type))
    {
        mVehicle->setControlInput(convertToEigen(aCmdMsg->data)); 
        setNewActiveState(StateMachine::States::THRUSTER_CONTROL); 
    }
    else if (CommandType::POSE == toEnum(aCmdMsg->type))
    {
        mVehicle->setGoalPose(convertToEigen(aCmdMsg->data)); 
        setNewActiveState(StateMachine::States::POSE_CONTROL);   
    }
    else if (CommandType::VELOCITY == toEnum(aCmdMsg->type))
    {
        mVehicle->setGoalVelocity(convertToEigen(aCmdMsg->data));
        setNewActiveState(StateMachine::States::VELOCITY_CONTROL); 
    }
    else if (CommandType::IDLE == toEnum(aCmdMsg->type))
    {
        setNewActiveState(StateMachine::States::IDLE); 
    }
}

Eigen::Vector3d CommandHandler::convertToEigen(abv_msgs::msg::AbvVec3 aVectorToConvert)
{
    Eigen::Vector3d vectorToReturn;
    vectorToReturn[0] = aVectorToConvert.x; 
    vectorToReturn[1] = aVectorToConvert.y; 
    vectorToReturn[2] = aVectorToConvert.yaw; 

    return vectorToReturn; 
}

void CommandHandler::setNewActiveState(StateMachine::States aNewState)
{
    auto currentState = mStateMachine->getActiveState(); 

    if(aNewState != currentState)
    { 
        mStateMachine->setActiveState(aNewState); 
    }
}

CommandHandler::CommandType CommandHandler::toEnum(const std::string& aType)
{
    CommandType enumToReturn; 

    if("Thruster" == aType || "thruster" == aType)
    {
        enumToReturn = CommandType::THRUSTER; 
    }
    else if ("Pose" == aType || "pose" == aType)
    {
        enumToReturn = CommandType::POSE; 
    }
    else if ("Velocity" == aType || "vel" == aType || "velocity" == aType)
    {
        enumToReturn = CommandType::VELOCITY; 
    }
    else if ("Idle" == aType || "idle" == aType)
    {
        enumToReturn = CommandType::IDLE; 
    }
    else if ("STOP" == aType || "stop" == aType || "Stop" == aType)
    {
        enumToReturn = CommandType::STOP; 
    }
    else
    {
        printf("Unsupported CommandType: %s", aType.c_str()); 
        enumToReturn = CommandType::NUM_TYPES; 
    }

    return enumToReturn; 
}

std::string CommandHandler::toString(CommandType aCmdType)
{
    std::string stringToReturn = ""; 
    switch (aCmdType)
    {
    case CommandType::IDLE:
        stringToReturn = "IDLE"; 
        break;
    case CommandType::THRUSTER: 
        stringToReturn = "THRUSTER_CONTROL";
        break; 
    case CommandType::POSE: 
        stringToReturn = "POSE_CONTROL"; 
        break;
    case CommandType::VELOCITY: 
        stringToReturn = "VELOCITY_CONTROL"; 
        break;
    case CommandType::STOP: 
        stringToReturn = "STOP"; 
        break; 
    default:
        break;
    }

    return stringToReturn; 
}