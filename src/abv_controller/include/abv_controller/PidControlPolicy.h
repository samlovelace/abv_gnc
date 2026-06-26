#ifndef PIDCONTROLPOLICY_H
#define PIDCONTROLPOLICY_H
 
#include <eigen3/Eigen/Dense> 
#include "abv_common/Configurations.h"
#include "abv_controller/IControlPolicy.hpp"
#include <chrono> 


class PidControlPolicy : public IControlPolicy
{ 
public:
    PidControlPolicy();
    ~PidControlPolicy();

    bool computeAction(const ControlContext& ctx, ActionContext& actionCtx) override;

private:
    ControlConfig mConfig; 
    std::chrono::steady_clock::time_point mPrevTime; 
    Eigen::Vector3d mPrevPoseError; 
    Eigen::Vector3d mPoseErrorIntegral; 
    
    
    Eigen::Vector3d PID(Eigen::Vector3d aPoseError); 
   
};
#endif //CONTROLLER_H