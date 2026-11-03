
#include "autobot_common/RosTopicManager.h"
#include "autobot_commander/CommanderComms.h"

CommanderComms::CommanderComms()
{
    rclcpp::init(0, nullptr); 
}

CommanderComms::~CommanderComms()
{

}

bool CommanderComms::start()
{
    auto topicManager = RosTopicManager::getInstance(); 

    topicManager->createPublisher<autobot_msgs::msg::ControllerCommand>("abv/controller/command");
    topicManager->createPublisher<autobot_msgs::msg::GuidanceCommand>("abv/guidance/command"); 
    
    topicManager->spinNode(); 

    return true; 
}

bool CommanderComms::stop()
{
    rclcpp::shutdown(); 
}