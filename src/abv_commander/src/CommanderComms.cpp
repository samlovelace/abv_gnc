
#include "common/RosTopicManager.h"
#include "abv_commander/CommanderComms.h"

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

    topicManager->createPublisher<abv_msgs::msg::AbvControllerCommand>("abv/controller/command");
    topicManager->createPublisher<abv_msgs::msg::AbvGuidanceCommand>("abv/guidance/command"); 
    
    topicManager->spinNode(); 

    return true; 
}

bool CommanderComms::stop()
{
    rclcpp::shutdown(); 
}