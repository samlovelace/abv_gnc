
#include "abv_common/RosTopicManager.h"
#include "abv_msgs/msg/abv_controller_command.hpp"

#include "abv_teleop/TeleopController.h"
#include "abv_teleop/ControlDeviceFactory.h"

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
    RosTopicManager::getInstance("abv_teleop")->createPublisher<abv_msgs::msg::AbvControllerCommand>("abv/controller/command");
    RosTopicManager::getInstance()->spinNode();

    auto controlDevice = ControlDeviceFactory::create("sfml");
    TeleopController tc(controlDevice);
    tc.run();

    rclcpp::shutdown();
}
