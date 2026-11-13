
#include <functional>
#include <kdl/jntarray.hpp>
#include <string>
#include <plog/Log.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "autobot_common/RosTopicManager.h"
#include "autobot_manip_common/ConfigManager.h"
#include "autobot_common/RateController.hpp"

#include "autobot_msgs/msg/manip_hardware_telemetry.hpp"

#include "autobot_manip_hardware/HardwareManager.h"
#include "autobot_manip_hardware/ManipulatorFactory.h"

HardwareManager::HardwareManager()
{

}

HardwareManager::~HardwareManager()
{

}

bool HardwareManager::init()
{
    RosTopicManager::getInstance()->createPublisher<autobot_msgs::msg::ManipHardwareTelemetry>("arm/hardware/telemetry");
    RosTopicManager::getInstance()->createSubscriber<autobot_msgs::msg::ManipHardwareGoal>("arm/hardware/goal",
        std::bind(&HardwareManager::goalCallback, this, std::placeholders::_1));

    const auto& config = ConfigManager::getInstance()->getConfig();
    mHasGripper = config.gripper.has_value();
    mNumArmJoints = static_cast<int>(config.manipulator.jointNames.size());

    if(mHasGripper)
    {
        RosTopicManager::getInstance()->createPublisher<autobot_msgs::msg::ManipHardwareTelemetry>("arm/hardware/gripper_telemetry");
        RosTopicManager::getInstance()->createSubscriber<autobot_msgs::msg::ManipHardwareGoal>("arm/hardware/gripper_goal",
            std::bind(&HardwareManager::gripperGoalCallback, this, std::placeholders::_1));
    }

    mComms = ManipulatorFactory::create();
    mComms->init();

    if(nullptr == mComms)
    {
        LOGE << "Failed to initialize hardware comms";
        return false;
    }

    return true;
}

void HardwareManager::run()
{
    RateController rate(20); // TODO: make config
    mRunning = true;

    KDL::JntArray pos, vel;
    std::vector<double> posV, velV;

    const std::string& name = ConfigManager::getInstance()->getConfig().manipName;

    autobot_msgs::msg::ManipHardwareTelemetry state;
    autobot_msgs::msg::ManipHardwareTelemetry gripperState;
    state.set__name(name);
    gripperState.set__name(name);

    while(mRunning)
    {
        rate.start();

        pos = mComms->getJointPositions();
        vel = mComms->getJointVelocities();
        posV.clear(); velV.clear();
        posV.resize(pos.rows()); velV.resize(vel.rows());

        for(int i = 0; i < (int)pos.rows(); i++)
        {
            posV[i] = pos(i);
            velV[i] = vel(i);
        }

        state.set__position(posV);
        state.set__velocity(velV);
        RosTopicManager::getInstance()->publishMessage("arm/hardware/telemetry", state);

        if(mHasGripper)
        {
            KDL::JntArray gripperPos = mComms->getGripperPositions();
            KDL::JntArray gripperVel = mComms->getGripperVelocities();

            std::vector<double> gripperPosV(gripperPos.rows());
            std::vector<double> gripperVelV(gripperVel.rows());

            for(int i = 0; i < (int)gripperPos.rows(); i++)
            {
                gripperPosV[i] = gripperPos(i);
                gripperVelV[i] = gripperVel(i);
            }

            gripperState.set__position(gripperPosV);
            gripperState.set__velocity(gripperVelV);
            RosTopicManager::getInstance()->publishMessage("arm/hardware/gripper_telemetry", gripperState);
        }

        rate.block();
    }
}

void HardwareManager::goalCallback(autobot_msgs::msg::ManipHardwareGoal::SharedPtr aGoal)
{
    // Arm joints only — the mobile base is owned by arm_controller
    if((int)aGoal->position.size() != mNumArmJoints)
    {
        LOGW << "Rejecting hardware goal of size " << aGoal->position.size() << ", expected " << mNumArmJoints;
        return;
    }

    KDL::JntArray goal(mNumArmJoints);
    for(int i = 0; i < mNumArmJoints; i++)
    {
        goal(i) = aGoal->position[i];
    }

    mComms->sendJointCommand(goal);
}

void HardwareManager::gripperGoalCallback(autobot_msgs::msg::ManipHardwareGoal::SharedPtr aGoal)
{
    KDL::JntArray goal(aGoal->position.size());
    for(int i = 0; i < (int) goal.rows(); i++)
    {
        goal(i) = aGoal->position[i];
    }

    mComms->sendGripperCommand(goal);
}
