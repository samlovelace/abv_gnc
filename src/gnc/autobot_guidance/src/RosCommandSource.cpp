
#include "autobot_guidance/RosCommandSource.h"
#include "autobot_common/RosTopicManager.h"

RosCommandSource::RosCommandSource(ICommandSink& aSink) : ICommandSource(aSink)
{

}

RosCommandSource::~RosCommandSource()
{

}

void RosCommandSource::listen()
{
    RosTopicManager::getInstance()->createSubscriber<autobot_msgs::msg::GuidanceCommand>("abv/guidance/command", 
                                                                                         std::bind(&RosCommandSource::commandCallback, 
                                                                                                   this, 
                                                                                                   std::placeholders::_1));

    RosTopicManager::getInstance()->spinNode();                                                                                        
    while(!RosTopicManager::getInstance()->isROSInitialized())
    {
        // wait to ensure ros is initialized 
    }
}

void RosCommandSource::commandCallback(autobot_msgs::msg::GuidanceCommand::SharedPtr aCmd)
{
    autobot_msgs::msg::Vec3 goal = aCmd->goal_state.position; 
    Waypoint wp(goal.x, goal.y, goal.yaw, "pose");  // TODO: support sending vel commands

    Command cmd; 
    cmd.mType = aCmd->type; 
    cmd.mDuration = aCmd->duration;
    cmd.mGoal = wp; 

    // TODO: when adding support for vel commands, update this too
    auto tol = aCmd->arrival_tolerance.position; 
    cmd.mArrivalTol << tol.x, tol.y, tol.yaw; 

    mCommandSink.onCommand(cmd); 
}

void RosCommandSource::stop()
{

}
