#ifndef COMMANDERCOMMS_H
#define COMMANDERCOMMS_H

#include "autobot_msgs/msg/controller_command.hpp"
#include "autobot_msgs/msg/guidance_command.hpp"

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