#ifndef HARDWAREMANAGER_H
#define HARDWAREMANAGER_H

#include "autobot_msgs/msg/manip_hardware_goal.hpp"
#include "autobot_manip_hardware/IManipComms.hpp"

#include <atomic>
#include <memory>

class HardwareManager
{
public:
    HardwareManager();
    ~HardwareManager();

    bool init();
    void run();

private:
    void goalCallback(autobot_msgs::msg::ManipHardwareGoal::SharedPtr aGoal);
    void gripperGoalCallback(autobot_msgs::msg::ManipHardwareGoal::SharedPtr aGoal);

private:
    std::shared_ptr<IManipComms> mComms;
    std::atomic_bool mRunning;
    bool mHasGripper;
    int mNumArmJoints;
};
#endif
