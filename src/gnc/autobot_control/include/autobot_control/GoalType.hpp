
#ifndef GOAL_TYPE_HPP
#define GOAL_TYPE_HPP

// mirrors autobot_goal_manager/GoalType.hpp and ControlContext.msg's goal_type
// field - kept as autobot_control's own local copy rather than a cross-package
// header dependency between otherwise-independent nodes.
enum class GoalType
{
    THRUSTER,
    POSE,
    VELOCITY,
    NUM_TYPES
};

#endif
