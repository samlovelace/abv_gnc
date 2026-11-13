
#include <cstdio>
#include "autobot_common/RosTopicManager.h"
#include "autobot_config/CommonConfigManager.h"
#include "autobot_common/HeartbeatPublisher.h"
#include "autobot_bridge/NavigationConvertor.h"
#include "autobot_bridge/ControllerStatusConvertor.h"
#include "autobot_bridge/WaypointConvertor.h"
#include "autobot_bridge/GazeboStateConvertor.h"

int main()
{
    CommonConfigManager::getInstance()->loadConfiguration();

    rclcpp::init(0, nullptr);
    RosTopicManager::getInstance("autobot_bridge");
    RosTopicManager::getInstance()->spinNode();

    // ABV -> Autonomy
    NavigationConvertor nav("abv/state", "robot/state");
    ControllerStatusConvertor status("abv/controller/status", "robot/vehicle/controller_status");

    // Autonomy -> ABV
    WaypointConvertor waypoint("robot/vehicle/waypoint", "abv/guidance/command");

    // Gazebo -> ABV Sim (external propagation feedback)
    GazeboStateConvertor gazeboState("mujoco/robot/state", "abv/sim/gazebo_state");

    HeartbeatPublisher heartbeat("bridge");

    while(true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1)); 
    }

    rclcpp::shutdown(); 
    return 0; 
}