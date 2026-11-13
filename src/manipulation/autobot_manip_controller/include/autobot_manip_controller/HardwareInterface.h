#ifndef HARDWAREINTERFACE_H
#define HARDWAREINTERFACE_H

#include <mutex>
#include <kdl/jntarray.hpp>

#include "autobot_msgs/msg/manip_hardware_telemetry.hpp"

class HardwareInterface
{
public:
    HardwareInterface();
    ~HardwareInterface();

    bool init();
    KDL::JntArray getJointPositions();
    KDL::JntArray getJointVelocities();
    void sendJointCommand(const KDL::JntArray &aCmd);

    KDL::JntArray getGripperPositions();
    KDL::JntArray getGripperVelocities();
    void sendGripperCommand(const KDL::JntArray &aCmd);

private:
    void stateCallback(autobot_msgs::msg::ManipHardwareTelemetry::SharedPtr aState);
    void gripperStateCallback(autobot_msgs::msg::ManipHardwareTelemetry::SharedPtr aState);

    std::mutex mStateMutex;
    KDL::JntArray mLatestPosition;
    KDL::JntArray mLatestVelocity;

    std::mutex mGripperStateMutex;
    KDL::JntArray mLatestGripperPosition;
    KDL::JntArray mLatestGripperVelocity;

};
#endif
