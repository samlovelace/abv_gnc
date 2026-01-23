#ifndef ROSCOMMANDSOURCE_H
#define ROSCOMMANDSOURCE_H
 
#include "common/RosTopicManager.h"
#include "abv_guidance/ICommandSource.hpp"

#include "robot_idl/msg/abv_guidance_command.hpp"
 
class RosCommandSource : public ICommandSource
{ 
public:
    explicit RosCommandSource(ICommandSink& aSink);
    ~RosCommandSource() override; 

    void listen() override; 
    void stop() override; 

private:

    void commandCallback(robot_idl::msg::AbvGuidanceCommand::SharedPtr aCmd); 
   
};
#endif //ROSCOMMANDSOURCE_H