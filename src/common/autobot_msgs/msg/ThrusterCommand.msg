# Published by autobot_goal_manager (direction/thruster/stop paths), autobot_control,
# or autobot_rl-cpp - whichever is currently computing control effort - and
# subscribed by autobot_thrusters. Mirrors ControllerCommand's envelope shape.

string type        # "direction" | "thruster"
bool is_global      # true if `data` is in the world frame; autobot_thrusters converts to body frame

Vec3 data         # control effort (fx, fy, tz), for type "direction"
string thrusters     # 8-char '0'/'1' string, for type "thruster"

# Identity of the control-law node that computed this command ("pid", "rl",
# etc). Left empty for autobot_goal_manager's direct writes (thruster/direction/
# stop paths), which always bypass the active-policy arbiter in
# autobot_thrusters. Non-empty values are gated against abv/policy/active -
# see autobot_policy_registry.
string policy_name
