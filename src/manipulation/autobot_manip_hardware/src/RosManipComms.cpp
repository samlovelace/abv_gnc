#include <algorithm>
#include <iterator>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "plog/Log.h"

#include "autobot_common/RosTopicManager.h"
#include "autobot_manip_hardware/RosManipComms.h"

RosManipComms::RosManipComms(const ConfigManager::Config& aConfig)
{
    mNumArmJoints = static_cast<int>(aConfig.manipulator.jointNames.size());
    mNumGripperJoints = aConfig.gripper
        ? static_cast<int>(aConfig.gripper->jointNames.size())
        : 0;

    mJointNames = aConfig.manipulator.jointNames;

    if(aConfig.gripper)
    {
        mJointNames.insert(
            mJointNames.end(),
            aConfig.gripper->jointNames.begin(),
            aConfig.gripper->jointNames.end()
        );
    }

    const int totalJoints = mNumArmJoints + mNumGripperJoints;

    mJointPositions.resize(totalJoints);
    mJointVelocities.resize(totalJoints);

    for(int i = 0; i < totalJoints; ++i)
    {
        mJointPositions(i) = 0.0;
        mJointVelocities(i) = 0.0;
    }

    mLastArmCmd.assign(mNumArmJoints, 0.0);
    mLastGripperCmd.assign(mNumGripperJoints, 0.0);
}

RosManipComms::~RosManipComms() = default;

bool RosManipComms::init()
{
    auto* ros = RosTopicManager::getInstance();

    ros->createSubscriber<sensor_msgs::msg::JointState>(
        "joint_states",
        std::bind(
            &RosManipComms::armStateCallback,
            this,
            std::placeholders::_1
        )
    );

    ros->createPublisher<sensor_msgs::msg::JointState>(
        "joint_commands"
    );

    return true;
}

KDL::JntArray RosManipComms::getJointPositions()
{
    std::lock_guard<std::mutex> lock(mJointPosMutex);

    KDL::JntArray armPos(mNumArmJoints);

    for(int i = 0; i < mNumArmJoints; ++i)
    {
        armPos(i) = mJointPositions(i);
    }

    return armPos;
}

KDL::JntArray RosManipComms::getJointVelocities()
{
    std::lock_guard<std::mutex> lock(mJointVelMutex);

    KDL::JntArray armVel(mNumArmJoints);

    for(int i = 0; i < mNumArmJoints; ++i)
    {
        armVel(i) = mJointVelocities(i);
    }

    return armVel;
}

KDL::JntArray RosManipComms::getGripperPositions()
{
    std::lock_guard<std::mutex> lock(mJointPosMutex);

    KDL::JntArray gripperPos(mNumGripperJoints);

    for(int i = 0; i < mNumGripperJoints; ++i)
    {
        gripperPos(i) = mJointPositions(mNumArmJoints + i);
    }

    return gripperPos;
}

KDL::JntArray RosManipComms::getGripperVelocities()
{
    std::lock_guard<std::mutex> lock(mJointVelMutex);

    KDL::JntArray gripperVel(mNumGripperJoints);

    for(int i = 0; i < mNumGripperJoints; ++i)
    {
        gripperVel(i) = mJointVelocities(mNumArmJoints + i);
    }

    return gripperVel;
}

void RosManipComms::sendJointCommand(const KDL::JntArray& aCmd)
{
    std::lock_guard<std::mutex> lock(mCmdMutex);

    if(!mCommandsInitialized)
    {
        PLOG_WARNING << "Ignoring arm command: waiting for initial joint state";
        return;
    }

    if(aCmd.rows() != mNumArmJoints)
    {
        PLOG_ERROR << "Arm command size mismatch";
        return;
    }

    for(int i = 0; i < mNumArmJoints; ++i)
    {
        mLastArmCmd[i] = aCmd(i);
    }

    publishCombinedCommand();
}

void RosManipComms::sendGripperCommand(const KDL::JntArray& aCmd)
{
    std::lock_guard<std::mutex> lock(mCmdMutex);

    if(!mCommandsInitialized)
    {
        PLOG_WARNING << "Ignoring gripper command: waiting for initial joint state";
        return;
    }

    if(aCmd.rows() != mNumGripperJoints)
    {
        PLOG_ERROR << "Gripper command size mismatch";
        return;
    }

    for(int i = 0; i < mNumGripperJoints; ++i)
    {
        mLastGripperCmd[i] = aCmd(i);
    }

    publishCombinedCommand();
}

// Caller must hold mCmdMutex.
void RosManipComms::publishCombinedCommand()
{
    std::vector<double> posCmd;

    posCmd.reserve(
        mLastArmCmd.size() + mLastGripperCmd.size()
    );

    posCmd.insert(
        posCmd.end(),
        mLastArmCmd.begin(),
        mLastArmCmd.end()
    );

    posCmd.insert(
        posCmd.end(),
        mLastGripperCmd.begin(),
        mLastGripperCmd.end()
    );

    sensor_msgs::msg::JointState cmd;
    cmd.name = mJointNames;
    cmd.position = std::move(posCmd);

    RosTopicManager::getInstance()->publishMessage<
        sensor_msgs::msg::JointState
    >("joint_commands", cmd);
}

void RosManipComms::armStateCallback(
    sensor_msgs::msg::JointState::SharedPtr msg)
{
    const std::size_t n = msg->name.size();

    if(msg->position.size() != n || msg->velocity.size() != n)
    {
        PLOG_ERROR << "Invalid JointState dimensions";
        return;
    }

    std::unordered_map<std::string, std::size_t> nameToIndex;

    for(std::size_t i = 0; i < n; ++i)
    {
        nameToIndex[msg->name[i]] = i;
    }

    // Require a complete joint state before initializing commands.
    if(!mCommandsInitialized)
    {
        for(const auto& name : mJointNames)
        {
            if(nameToIndex.find(name) == nameToIndex.end())
            {
                return;
            }
        }
    }

    {
        std::scoped_lock lock(mJointPosMutex, mJointVelMutex);

        for(std::size_t i = 0; i < mJointNames.size(); ++i)
        {
            auto it = nameToIndex.find(mJointNames[i]);

            if(it == nameToIndex.end())
            {
                continue;
            }

            const std::size_t msgIndex = it->second;

            mJointPositions(i) = msg->position[msgIndex];
            mJointVelocities(i) = msg->velocity[msgIndex];
        }
    }

    // Initialize the command buffers to the actual joint positions.
    // Lock ordering is consistent: state locks are released before cmd lock.
    {
        std::lock_guard<std::mutex> lock(mCmdMutex);

        if(!mCommandsInitialized)
        {
            for(int i = 0; i < mNumArmJoints; ++i)
            {
                mLastArmCmd[i] = msg->position[
                    nameToIndex.at(mJointNames[i])
                ];
            }

            for(int i = 0; i < mNumGripperJoints; ++i)
            {
                const int index = mNumArmJoints + i;

                mLastGripperCmd[i] = msg->position[
                    nameToIndex.at(mJointNames[index])
                ];
            }

            mCommandsInitialized = true;

            PLOG_INFO << "RosManipComms initialized from joint states";
        }
    }
}