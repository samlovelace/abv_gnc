
#include <cstdio> 
#include "common/RosTopicManager.h"
#include "abv_bridge/NavigationConvertor.h"
#include "abv_bridge/ControllerStatusConvertor.h"
#include "abv_bridge/WaypointConvertor.h"

int main()
{
    rclcpp::init(0, nullptr); 
    RosTopicManager::getInstance("abv_bridge");
    RosTopicManager::getInstance()->spinNode();  

    // ABV -> Autonomy 
    NavigationConvertor nav("abv/state", "robot/state");
    ControllerStatusConvertor status("abv/controller/status", "robot/vehicle/controller_status"); 

    // Autonomy -> ABV
    WaypointConvertor waypoint("robot/vehicle/waypoint", "abv/guidance/command");

    while(true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1)); 
    }

    rclcpp::shutdown(); 
    return 0; 
}