#ifndef CONTROLLAWHANDLER_H
#define CONTROLLAWHANDLER_H

#include <memory>
#include "abv_control/IControlLaw.hpp"
#include "abv_msgs/msg/abv_control_context.hpp"

/**
 * @brief ControlLawHandler subscribes to abv/controller/context, runs the
 *        configured IControlLaw against each message, and publishes the
 *        resulting control effort on abv/thrusters/command. Instantiates
 *        the IControlLaw implementation selected by ControlLawConfig.mLawType
 *        (PID today; a new law is a new IControlLaw implementation plus one
 *        more branch in the constructor).
 */
class ControlLawHandler
{
public:
    ControlLawHandler();
    ~ControlLawHandler();

private:
    void contextCallback(abv_msgs::msg::AbvControlContext::SharedPtr aCtxMsg);

private:
    std::unique_ptr<IControlLaw> mControlLaw;
};
#endif // CONTROLLAWHANDLER_H
