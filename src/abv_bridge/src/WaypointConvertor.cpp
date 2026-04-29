
#include "abv_bridge/WaypointConvertor.h"
#include "abv_msgs/msg/abv_guidance_command.hpp"
#include "abv_common/RosTopicManager.h"

WaypointConvertor::WaypointConvertor(const std::string& anIncomingTopic, const std::string& anOutgoingTopic) : 
    mIncomingTopic(anIncomingTopic), mOutgoingTopic(anOutgoingTopic)
{
    RosTopicManager::getInstance()->createPublisher<abv_msgs::msg::AbvGuidanceCommand>(anOutgoingTopic); 
    RosTopicManager::getInstance()->createSubscriber<robot_idl::msg::VehicleWaypoint>(mIncomingTopic, 
            std::bind(&WaypointConvertor::convert, this, std::placeholders::_1)); 

}

WaypointConvertor::~WaypointConvertor()
{

}

void WaypointConvertor::convert(robot_idl::msg::VehicleWaypoint::SharedPtr aWaypoint)
{
    // Convert goal state 
    abv_msgs::msg::AbvVec3 pos; 
    pos.set__x(aWaypoint->position.x); 
    pos.set__y(aWaypoint->position.y); 
    pos.set__yaw(aWaypoint->euler.yaw); 
    abv_msgs::msg::AbvVec3 vel; 
    vel.set__x(0.0); 
    vel.set__y(0.0); 
    vel.set__yaw(0.0); 
    abv_msgs::msg::AbvState goalState; 
    goalState.set__position(pos); 
    goalState.set__velocity(vel); 

    // convert arrival_tolerance 
    abv_msgs::msg::AbvVec3 pos_tol; 
    pos_tol.set__x(aWaypoint->position_tolerance.x); 
    pos_tol.set__y(aWaypoint->position_tolerance.y); 
    pos_tol.set__yaw(aWaypoint->euler_tolerance.yaw); 
    abv_msgs::msg::AbvVec3 vel_tol; 
    vel_tol.set__x(0.1); 
    vel_tol.set__y(0.1); 
    vel_tol.set__yaw(0.1); 
    abv_msgs::msg::AbvState arrivalTol; 
    arrivalTol.set__position(pos_tol); 
    arrivalTol.set__velocity(vel_tol); 

    // populate final command 
    abv_msgs::msg::AbvGuidanceCommand cmd; 
    cmd.set__goal_state(goalState);
    cmd.set__arrival_tolerance(arrivalTol);  
    cmd.set__type("line"); 
    cmd.set__duration(aWaypoint->execution_duration); // seconds 

    RosTopicManager::getInstance()->publishMessage(mOutgoingTopic, cmd); 
}