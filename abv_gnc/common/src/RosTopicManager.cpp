
#include "common/RosTopicManager.h"

RosTopicManager::RosTopicManager(const std::string& aNodeName) : Node(aNodeName)
{
    
}

RosTopicManager::~RosTopicManager()
{
    rclcpp::shutdown();
}

void RosTopicManager::spinNode()
{
    std::thread([this]() {
        rclcpp::spin(this->get_node_base_interface());
    }).detach();
}