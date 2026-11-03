
#include "autobot_common/RosTopicManager.h"
#include "autobot_msgs/msg/controller_command.hpp"

#include "autobot_teleop/TeleopController.h"
#include "autobot_teleop/ControlDeviceFactory.h"

// Signal handler function
void signalHandler(int signal) {

    // LOGD  << "\n" << R"(
    // _________________________
    // |                       |
    // |   SHUTTING DOWN...    |
    // |_______________________|
    //         \   ^__^
    //          \  (oo)\_______
    //             (__)\       )\/\
    //                 ||----w |
    //                 ||     ||)";
    exit(0); // Exit the program
}

int main()
{
    std::signal(SIGINT, signalHandler);

    rclcpp::init(0, nullptr);
    RosTopicManager::getInstance("autobot_teleop")->createPublisher<autobot_msgs::msg::ControllerCommand>("abv/controller/command");
    RosTopicManager::getInstance()->spinNode();

    auto controlDevice = ControlDeviceFactory::create("sfml");
    TeleopController tc(controlDevice);
    tc.run();

    rclcpp::shutdown();
}
