#ifndef ROSMANIPCOMMS_H
#define ROSMANIPCOMMS_H

#include <mutex>
#include <chrono>

#include <sensor_msgs/msg/joint_state.hpp>

#include "autobot_manip_hardware/IManipComms.hpp"
#include "autobot_config/ManipConfigManager.h"

class RosManipComms : public IManipComms
{
public:
    RosManipComms(const ManipConfigManager::Config& aConfig);
    ~RosManipComms() override;

    bool init() override;
    KDL::JntArray getJointPositions() override;
    KDL::JntArray getJointVelocities() override;
    void sendJointCommand(const KDL::JntArray &aCmd) override;

    KDL::JntArray getGripperPositions() override;
    KDL::JntArray getGripperVelocities() override;
    void sendGripperCommand(const KDL::JntArray &aCmd) override;

private:
    void publishCombinedCommand();

private:

    void armStateCallback(sensor_msgs::msg::JointState::SharedPtr msg);

    // Arm only — the mobile base is owned by arm_controller.
    // State arrays are laid out [arm..., gripper...]
    int mNumArmJoints;
    int mNumGripperJoints;
    std::vector<std::string> mJointNames;

    std::mutex mJointPosMutex;
    KDL::JntArray mJointPositions;

    std::mutex mJointVelMutex;
    KDL::JntArray mJointVelocities;

    // Arm and gripper share one combined command array on the wire
    // (UR/joint_commands) but arrive independently via sendJointCommand /
    // sendGripperCommand — cache both and republish the merge each time.
    std::mutex mCmdMutex;
    std::vector<double> mLastArmCmd;
    std::vector<double> mLastGripperCmd;

    bool mCommandsInitialized = false;

};
#endif //ROSMANIPCOMMS_H
