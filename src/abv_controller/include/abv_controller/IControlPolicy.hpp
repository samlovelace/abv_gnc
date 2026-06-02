#ifndef ICONTROLPOLICY_HPP
#define ICONTROLPOLICY_HPP
 
#include <Eigen/Dense>
#include "abv_controller/GoalType.hpp"

struct ControlContext 
{
    Eigen::Vector3d  currentPose;
    Eigen::Vector3d  currentVelocity;
    Eigen::Vector3d  goal;
    Eigen::Vector3d  error;
    GoalType         goalType;
};

class IControlPolicy
{
public:
    virtual ~IControlPolicy() = default;
    virtual Eigen::Vector3d computeAction(const ControlContext& ctx) = 0;
};
#endif //ICONTROLPOLICY_HPP 