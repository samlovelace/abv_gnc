#ifndef COMMANDERCOMMS_H
#define COMMANDERCOMMS_H

#include "autobot_msgs/msg/controller_command.hpp"
#include "autobot_msgs/msg/guidance_command.hpp"
#include "autobot_msgs/msg/joint_position_waypoint.hpp"
#include "autobot_msgs/msg/execute_pick_command.hpp"

class CommanderComms 
{ 
public:
    CommanderComms();
    ~CommanderComms();

    bool start(); 
    bool stop(); 

private:
   
};
#endif //COMMANDERCOMMS_H   