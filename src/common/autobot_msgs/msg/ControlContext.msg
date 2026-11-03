# Published by autobot_goal_manager once per state-machine tick while in
# POSE_CONTROL/VELOCITY_CONTROL, so any control-law node (autobot_control,
# autobot_rl-cpp) can compute a control input without autobot_goal_manager needing
# to know which/how many control-law nodes are listening.

Vec3 pose
Vec3 velocity
Vec3 goal
Vec3 error

# mirrors GoalType.hpp
uint8 POSE=0
uint8 VELOCITY=1

uint8 goal_type
