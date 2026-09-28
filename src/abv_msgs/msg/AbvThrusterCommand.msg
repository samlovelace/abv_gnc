# Published by abv_goal_manager (direction/thruster/stop paths), abv_control,
# or abv_rl-cpp - whichever is currently computing control effort - and
# subscribed by abv_thrusters. Mirrors AbvControllerCommand's envelope shape.

string type        # "direction" | "thruster"
bool is_global      # true if `data` is in the world frame; abv_thrusters converts to body frame

AbvVec3 data         # control effort (fx, fy, tz), for type "direction"
string thrusters     # 8-char '0'/'1' string, for type "thruster"
