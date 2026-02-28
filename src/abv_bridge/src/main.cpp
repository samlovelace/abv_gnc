
#include <cstdio> 
#include "common/RosTopicManager.h"
#include "abv_bridge/NavigationConvertor.h"

int main()
{
    rclcpp::init(0, nullptr); 
    RosTopicManager::getInstance("abv_bridge");
    RosTopicManager::getInstance()->spinNode();  

    NavigationConvertor nav("abv/state", "robot/state"); 

    while(true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1)); 
    }

    rclcpp::shutdown(); 
    return 0; 
}