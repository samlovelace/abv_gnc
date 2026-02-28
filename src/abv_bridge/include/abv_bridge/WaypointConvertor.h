#ifndef WAYPOINTCONVERTOR_H
#define WAYPOINTCONVERTOR_H
 
#include "robot_idl/msg/vehicle_waypoint.hpp"

 
class WaypointConvertor 
{ 
public:
    WaypointConvertor(const std::string& anIncomingTopic, const std::string& anOutgoingTopic);
    ~WaypointConvertor();

private:
    void convert(robot_idl::msg::VehicleWaypoint::SharedPtr aWaypoint); 

private: 
    std::string mIncomingTopic; 
    std::string mOutgoingTopic; 

};
#endif //WAYPOINTCONVERTOR_H