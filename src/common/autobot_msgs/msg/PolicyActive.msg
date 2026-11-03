# Published periodically by autobot_policy_registry on abv/policy/active -
# the single source of truth for which control-policy node is currently
# allowed to drive abv/thrusters/command. Empty policy_name means no policy
# has been activated yet (autobot_thrusters' arbiter drops every policy-sourced
# ThrusterCommand in that state - only autobot_goal_manager's direct writes,
# which carry an empty policy_name themselves, get through).

string policy_name
