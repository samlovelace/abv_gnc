#ifndef WAYPOINTCONVERTOR_H
#define WAYPOINTCONVERTOR_H
 
#include "ptera_msgs/msg/vehicle_waypoint.hpp"

 
class WaypointConvertor 
{ 
public:
    WaypointConvertor(const std::string& anIncomingTopic, const std::string& anOutgoingTopic);
    ~WaypointConvertor();

private:
    void convert(ptera_msgs::msg::VehicleWaypoint::SharedPtr aWaypoint); 

private: 
    std::string mIncomingTopic; 
    std::string mOutgoingTopic; 

};
#endif //WAYPOINTCONVERTOR_H