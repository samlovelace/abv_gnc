
#include "abv_guidance/RosCommandSource.h"
#include "common/RosTopicManager.h"

RosCommandSource::RosCommandSource(ICommandSink& aSink) : ICommandSource(aSink)
{

}

RosCommandSource::~RosCommandSource()
{

}

void RosCommandSource::listen()
{
    RosTopicManager::getInstance()->createSubscriber<abv_msgs::msg::AbvGuidanceCommand>("abv/guidance/command", 
                                                                                         std::bind(&RosCommandSource::commandCallback, 
                                                                                                   this, 
                                                                                                   std::placeholders::_1));

    RosTopicManager::getInstance()->spinNode();                                                                                        
    while(!RosTopicManager::getInstance()->isROSInitialized())
    {
        // wait to ensure ros is initialized 
    }
}

void RosCommandSource::commandCallback(abv_msgs::msg::AbvGuidanceCommand::SharedPtr aCmd)
{
    Command cmd; 
    cmd.mType = aCmd->type; 
    cmd.mDuration = aCmd->duration; 

    mCommandSink.onCommand(cmd); 
}

void RosCommandSource::stop()
{

}
