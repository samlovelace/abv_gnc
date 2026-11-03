#ifndef ROSCOMMANDSOURCE_H
#define ROSCOMMANDSOURCE_H
 
#include "autobot_common/RosTopicManager.h"
#include "autobot_guidance/ICommandSource.hpp"

#include "autobot_msgs/msg/guidance_command.hpp"
 
class RosCommandSource : public ICommandSource
{ 
public:
    explicit RosCommandSource(ICommandSink& aSink);
    ~RosCommandSource() override; 

    void listen() override; 
    void stop() override; 

private:

    void commandCallback(autobot_msgs::msg::GuidanceCommand::SharedPtr aCmd); 
   
};
#endif //ROSCOMMANDSOURCE_H