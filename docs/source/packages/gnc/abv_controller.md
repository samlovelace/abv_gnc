# abv_controller (removed)

`abv_controller` no longer exists. Its responsibilities were split across four packages:

| Former responsibility | Now in |
|---|---|
| Command intake (`abv/controller/command`), state machine, goal/error tracking, arrival status | [`abv_goal_manager`](abv_goal_manager.md) |
| PID control law | [`abv_control`](abv_control.md) |
| Thruster allocation, GPIO/UDP drivers | [`abv_thrusters`](abv_thrusters.md) |
| Selecting which control policy drives the vehicle (formerly `Control.ControlPolicy` and the `AbvControlAction` service) | [`abv_policy_registry`](abv_policy_registry.md), switchable at runtime |

The topic names `abv/controller/command`, `abv/controller/status`, and the new `abv/controller/context` were kept, and are now served by `abv_goal_manager`.
