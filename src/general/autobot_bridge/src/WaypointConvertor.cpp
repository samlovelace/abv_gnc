
#include "autobot_bridge/WaypointConvertor.h"
#include "autobot_msgs/msg/guidance_command.hpp"
#include "autobot_common/RosTopicManager.h"

WaypointConvertor::WaypointConvertor(const std::string& anIncomingTopic, const std::string& anOutgoingTopic) : 
    mIncomingTopic(anIncomingTopic), mOutgoingTopic(anOutgoingTopic)
{
    RosTopicManager::getInstance()->createPublisher<autobot_msgs::msg::GuidanceCommand>(anOutgoingTopic); 
    RosTopicManager::getInstance()->createSubscriber<ptera_msgs::msg::VehicleWaypoint>(mIncomingTopic, 
            std::bind(&WaypointConvertor::convert, this, std::placeholders::_1)); 

}

WaypointConvertor::~WaypointConvertor()
{

}

void WaypointConvertor::convert(ptera_msgs::msg::VehicleWaypoint::SharedPtr aWaypoint)
{
    // Convert goal state 
    autobot_msgs::msg::Vec3 pos; 
    pos.set__x(aWaypoint->position.x); 
    pos.set__y(aWaypoint->position.y); 
    pos.set__yaw(aWaypoint->euler.yaw); 
    autobot_msgs::msg::Vec3 vel; 
    vel.set__x(0.0); 
    vel.set__y(0.0); 
    vel.set__yaw(0.0); 
    autobot_msgs::msg::State goalState; 
    goalState.set__position(pos); 
    goalState.set__velocity(vel); 

    // convert arrival_tolerance 
    autobot_msgs::msg::Vec3 pos_tol; 
    pos_tol.set__x(aWaypoint->position_tolerance.x); 
    pos_tol.set__y(aWaypoint->position_tolerance.y); 
    pos_tol.set__yaw(aWaypoint->euler_tolerance.yaw); 
    autobot_msgs::msg::Vec3 vel_tol; 
    vel_tol.set__x(0.1); 
    vel_tol.set__y(0.1); 
    vel_tol.set__yaw(0.1); 
    autobot_msgs::msg::State arrivalTol; 
    arrivalTol.set__position(pos_tol); 
    arrivalTol.set__velocity(vel_tol); 

    // populate final command 
    autobot_msgs::msg::GuidanceCommand cmd; 
    cmd.set__goal_state(goalState);
    cmd.set__arrival_tolerance(arrivalTol);  
    cmd.set__type("line"); 
    cmd.set__duration(aWaypoint->execution_duration); // seconds 

    RosTopicManager::getInstance()->publishMessage(mOutgoingTopic, cmd); 
}