#ifndef COMMANDERCOMMS_H
#define COMMANDERCOMMS_H

#include "abv_msgs/msg/abv_command.hpp"
#include "abv_msgs/msg/abv_guidance_command.hpp"

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