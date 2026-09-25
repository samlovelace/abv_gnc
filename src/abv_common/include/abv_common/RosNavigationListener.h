#ifndef ROSNAVIGATIONLISTENER_H
#define ROSNAVIGATIONLISTENER_H

#include "abv_msgs/msg/abv_state.hpp"
#include "abv_common/Watchdog.h"
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <atomic>

class RosNavigationListener
{
public:
    RosNavigationListener(std::function<void(abv_msgs::msg::AbvState::SharedPtr)> aCallback = nullptr);
    ~RosNavigationListener();

    Eigen::Vector3d getCurrentPose();
    Eigen::Vector3d getCurrentVel();

    bool hasAcquiredStateData();
    Eigen::Matrix<double, 6, 1> getCurrentState();
    void setState(const Eigen::Matrix<double, 6, 1>& aState);

    // true iff a abv/state message has arrived within the configured
    // NavDataTimeout (catches full topic/node dropout)
    bool isNavDataFresh();

    // true iff the most recently received abv/state message reported
    // valid=true (catches upstream EKF free-run/dead-reckoning)
    bool isNavDataValid();

    // convenience: fresh && valid
    bool isNavOk();

private:
    void stateCallback(const abv_msgs::msg::AbvState::SharedPtr aMsg);

private:
    Eigen::Matrix<double, 6, 1> mCurrentState;
    std::mutex mCurrentStateMutex;
    bool mAcquiredState;

    Watchdog mWatchdog;
    std::atomic<bool> mIsFresh{false};
    std::atomic<bool> mLastMsgValid{false};
    double mNavDataTimeout;
};
#endif

