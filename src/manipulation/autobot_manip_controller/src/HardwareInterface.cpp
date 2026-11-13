#include "plog/Log.h"

#include "autobot_common/RosTopicManager.h"

#include "autobot_msgs/msg/manip_hardware_goal.hpp"

#include "autobot_manip_controller/HardwareInterface.h"

HardwareInterface::HardwareInterface()
{

}

HardwareInterface::~HardwareInterface()
{

}

bool HardwareInterface::init()
{
    // Create publishers and subscribers to interface with autobot_manip_hardware
    RosTopicManager::getInstance()->createSubscriber<autobot_msgs::msg::ManipHardwareTelemetry>("arm/hardware/telemetry",
                                                                 std::bind(&HardwareInterface::stateCallback,
                                                                           this,
                                                                           std::placeholders::_1));

    RosTopicManager::getInstance()->createPublisher<autobot_msgs::msg::ManipHardwareGoal>("arm/hardware/goal");

    // Parallel gripper channel — unconditionally created; harmless if no
    // gripper is configured (autobot_manip_hardware simply never publishes state on it).
    RosTopicManager::getInstance()->createSubscriber<autobot_msgs::msg::ManipHardwareTelemetry>("arm/hardware/gripper_telemetry",
                                                                 std::bind(&HardwareInterface::gripperStateCallback,
                                                                           this,
                                                                           std::placeholders::_1));

    RosTopicManager::getInstance()->createPublisher<autobot_msgs::msg::ManipHardwareGoal>("arm/hardware/gripper_goal");
}

KDL::JntArray HardwareInterface::getJointPositions()
{
    std::lock_guard<std::mutex> lock(mStateMutex);
    return mLatestPosition;
}

KDL::JntArray HardwareInterface::getJointVelocities()
{
    std::lock_guard<std::mutex> lock(mStateMutex);
    return mLatestVelocity;
}

void HardwareInterface::sendJointCommand(const KDL::JntArray &aCmd)
{
    std::vector<double> goal(aCmd.rows());
    for(int i = 0; i < goal.size(); i++)
    {
        //LOGV << "Sending joint command: aCmd(" << i << "): " << aCmd(i);
        goal[i] = aCmd(i);
    }

    autobot_msgs::msg::ManipHardwareGoal goalMsg;
    goalMsg.set__position(goal);

    RosTopicManager::getInstance()->publishMessage("arm/hardware/goal", goalMsg);
}

void HardwareInterface::stateCallback(autobot_msgs::msg::ManipHardwareTelemetry::SharedPtr aState)
{
    KDL::JntArray pos(aState->position.size());
    KDL::JntArray vel(aState->velocity.size());

    for(int i = 0; i < pos.rows(); i++)
    {
        pos(i) = aState->position[i];
        vel(i) = aState->velocity[i];
    }

    std::lock_guard<std::mutex> lock(mStateMutex);
    mLatestPosition = pos;
    mLatestVelocity = vel;
}

KDL::JntArray HardwareInterface::getGripperPositions()
{
    std::lock_guard<std::mutex> lock(mGripperStateMutex);
    return mLatestGripperPosition;
}

KDL::JntArray HardwareInterface::getGripperVelocities()
{
    std::lock_guard<std::mutex> lock(mGripperStateMutex);
    return mLatestGripperVelocity;
}

void HardwareInterface::sendGripperCommand(const KDL::JntArray &aCmd)
{
    std::vector<double> goal(aCmd.rows());
    for(int i = 0; i < goal.size(); i++)
    {
        goal[i] = aCmd(i);
    }

    autobot_msgs::msg::ManipHardwareGoal goalMsg;
    goalMsg.set__position(goal);

    RosTopicManager::getInstance()->publishMessage("arm/hardware/gripper_goal", goalMsg);
}

void HardwareInterface::gripperStateCallback(autobot_msgs::msg::ManipHardwareTelemetry::SharedPtr aState)
{
    KDL::JntArray pos(aState->position.size());
    KDL::JntArray vel(aState->velocity.size());

    for(int i = 0; i < pos.rows(); i++)
    {
        pos(i) = aState->position[i];
        vel(i) = aState->velocity[i];
    }

    std::lock_guard<std::mutex> lock(mGripperStateMutex);
    mLatestGripperPosition = pos;
    mLatestGripperVelocity = vel;
}
