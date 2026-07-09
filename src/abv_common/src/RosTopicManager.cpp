
#include "abv_common/RosTopicManager.h"

RosTopicManager::RosTopicManager(const std::string& aNodeName) : Node(aNodeName)
{
    
}

RosTopicManager::~RosTopicManager()
{
    rclcpp::shutdown();
}

void RosTopicManager::spinNode()
{
    // idempotent: several call sites (main() and various constructors, e.g.
    // CommandHandler) may each try to start spinning - only the first should
    // actually spawn an executor thread, since spinning the same node from
    // two concurrent rclcpp::spin() calls is undefined behavior.
    if (mSpinning.exchange(true))
    {
        return;
    }

    std::thread([this]() {
        rclcpp::spin(this->get_node_base_interface());
    }).detach();
}