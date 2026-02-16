#ifndef ROSNAVIGATIONLISTENER_H
#define ROSNAVIGATIONLISTENER_H

#include "abv_msgs/msg/abv_state.hpp"
#include <eigen3/Eigen/Dense>
#include <mutex> 

class RosNavigationListener
{
public:
    RosNavigationListener();
    ~RosNavigationListener();

    Eigen::Vector3d getCurrentPose(); 
    Eigen::Vector3d getCurrentVel(); 

    bool hasAcquiredStateData(); 
    Eigen::Matrix<double, 12, 1> getCurrentState();
    void setState(const Eigen::Matrix<double, 12, 1>& aState); 

private: 

    void stateCallback(const abv_msgs::msg::AbvState::SharedPtr aMsg); 

    Eigen::Matrix<double, 12, 1> mCurrentState; 
    std::mutex mCurrentStateMutex; 

    bool mAcquiredState; 
};
#endif 

