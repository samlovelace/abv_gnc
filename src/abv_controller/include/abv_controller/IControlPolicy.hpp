#ifndef ICONTROLPOLICY_HPP
#define ICONTROLPOLICY_HPP
 
#include <Eigen/Dense>
#include "abv_controller/GoalType.hpp"

struct ControlContext 
{
    // Inputs
    Eigen::Vector3d  currentPose;
    Eigen::Vector3d  currentVelocity;
    Eigen::Vector3d  goal;
    Eigen::Vector3d  error;
    GoalType         goalType;
}; 
struct ActionContext
{
    // Outputs
    Eigen::Vector3d  controlInput; 
    bool             isGlobal;
};

class IControlPolicy
{
public:
    virtual ~IControlPolicy() = default;
    virtual bool computeAction(const ControlContext& ctx, ActionContext& actionCtx) = 0;
};
#endif //ICONTROLPOLICY_HPP 