# autobot_controller (removed)

`autobot_controller` no longer exists. Its responsibilities were split across four packages:

| Former responsibility                                                                                                   | Now in                                                                         |
| ----------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------ |
| Command intake (`abv/controller/command`), state machine, goal/error tracking, arrival status                           | [`autobot_goal_manager`](autobot_goal_manager.md)                              |
| PID control law                                                                                                         | [`autobot_control`](autobot_control.md)                                        |
| Thruster allocation, GPIO/UDP drivers                                                                                   | [`autobot_thrusters`](autobot_thrusters.md)                                    |
| Selecting which control policy drives the vehicle (formerly `Control.ControlPolicy` and the `AbvControlAction` service) | [`autobot_policy_registry`](autobot_policy_registry.md), switchable at runtime |

The topic names `abv/controller/command`, `abv/controller/status`, and the new `abv/controller/context` were kept, and are now served by `autobot_goal_manager`.
