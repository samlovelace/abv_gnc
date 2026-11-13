#ifndef PIDCONTROLLAW_H
#define PIDCONTROLLAW_H

#include <eigen3/Eigen/Dense>
#include "autobot_config/GncConfigurations.h"
#include "autobot_control/IControlLaw.hpp"
#include <chrono>


class PidControlLaw : public IControlLaw
{
public:
    PidControlLaw();
    ~PidControlLaw();

    bool computeAction(const ControlContext& ctx, ActionContext& actionCtx) override;
    void reset() override;

private:
    ControlLawConfig mConfig;
    std::chrono::steady_clock::time_point mPrevTime;
    Eigen::Vector3d mPrevPoseError;
    Eigen::Vector3d mPoseErrorIntegral;


    Eigen::Vector3d PID(Eigen::Vector3d aPoseError);

};
#endif //PIDCONTROLLAW_H
