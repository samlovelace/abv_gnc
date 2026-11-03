
#include <vector> 
#include <iostream>
#include <string> 
#include <vector> 
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "autobot_commander/InputHandler.h"
#include "autobot_common/RosTopicManager.h"
#include "autobot_commander/CommanderComms.h"

InputHandler::InputHandler()
{
    mPackagePath = ament_index_cpp::get_package_share_directory("autobot_commander");
    std::cout << "Package path: " << mPackagePath << std::endl; 
}

InputHandler::~InputHandler()
{

}

void InputHandler::handle(const std::string& anInput)
{
    if ("pose" == anInput)
    {
        std::array<double, 3> vehPose;
        std::string vehPoseStr;

        std::cout << "Enter goal pose (x, y, yaw): ";
        std::getline(std::cin, vehPoseStr);

        std::istringstream iss(vehPoseStr);
        for (int i = 0; i < 3; ++i)
        {
            if (!(iss >> vehPose[i]))
            {
                std::cerr << "Invalid input. Please enter three space-separated numbers." << std::endl;
                return;
            }
        }

        autobot_msgs::msg::Vec3 data; 
        data.set__x(vehPose[0]); 
        data.set__y(vehPose[1]); 
        data.set__yaw(vehPose[2]); 

        autobot_msgs::msg::ControllerCommand cmd; 
        cmd.set__data(data); 
        cmd.set__type("pose");
        
        RosTopicManager::getInstance()->publishMessage<autobot_msgs::msg::ControllerCommand>("abv/controller/command", cmd); 
        std::cout << "Published goal waypoint (x y yaw): " << vehPose[0] << ", " << vehPose[1] << ", " << vehPose[2] << std::endl; 
    }
    else if ("vel" == anInput)
    {
        std::array<double, 3> vehPose;
        std::string vehPoseStr;

        std::cout << "Enter goal velocity (xd, yd, yawd): ";
        std::getline(std::cin, vehPoseStr);

        std::istringstream iss(vehPoseStr);
        for (int i = 0; i < 3; ++i)
        {
            if (!(iss >> vehPose[i]))
            {
                std::cerr << "Invalid input. Please enter three space-separated numbers." << std::endl;
                return;
            }
        }

        autobot_msgs::msg::Vec3 data; 
        data.set__x(vehPose[0]); 
        data.set__y(vehPose[1]); 
        data.set__yaw(vehPose[2]); 

        autobot_msgs::msg::ControllerCommand cmd; 
        cmd.set__data(data); 
        cmd.set__type("velocity");
        
        RosTopicManager::getInstance()->publishMessage<autobot_msgs::msg::ControllerCommand>("abv/controller/command", cmd); 
        std::cout << "Published goal velocity waypoint (xd yd yawd): " << vehPose[0] << ", " << vehPose[1] << ", " << vehPose[2] << std::endl;
    }
    else if ("stop" == anInput)
    {
        autobot_msgs::msg::Vec3 data; 
        data.set__x(0); 
        data.set__y(0); 
        data.set__yaw(0); 

        autobot_msgs::msg::ControllerCommand cmd; 
        cmd.set__data(data); 
        cmd.set__type("STOP");
        
        RosTopicManager::getInstance()->publishMessage<autobot_msgs::msg::ControllerCommand>("abv/controller/command", cmd); 
    }
    else if ("path" == anInput)
    {
        std::string trajType;
        std::string duration; 

        std::cout << GREEN << "Type: "; 
        std::getline(std::cin, trajType);

        std::cout << GREEN << "Duration (s): "; 
        std::getline(std::cin, duration); 
        double dur = std::stod(duration); 

        autobot_msgs::msg::GuidanceCommand cmd; 
        cmd.set__type(trajType); 
        cmd.set__duration(dur); 

        RosTopicManager::getInstance()->publishMessage<autobot_msgs::msg::GuidanceCommand>("abv/guidance/command", cmd); 
    }
    else if (anInput == "help" || anInput == "--help" || anInput == "-h") 
    {
        std::cout << R"(
        ABV Commander Tool
        -----------------------------
        Interact with the air-bearing vehicle by sending pose or velocity waypoints. 
        
        Available Commands:
        
        pose:       command the abv to a desired x, y, yaw pose 
        vel:        command the abv to a desired x, y, yaw velocity 
        stop:       command the abv to zero velocity on x, y, yaw axes 
        path:       command the abv to follow a path sent by the guidance node 
        
        Type 'help' or '--help' to show this message again.
        )" << std::endl;
        std::string test; 
        std::getline(std::cin, test); 
    }       
    else{
        std::cout << RED << "Unsupported command!" << std::endl; 
    }
}
