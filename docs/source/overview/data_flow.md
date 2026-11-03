# Data Flow

This page lists every topic and service used across the ABV stack: who publishes/calls it, who subscribes/serves it, and which [`autobot_msgs`](../packages/interface/autobot_msgs.md) type it carries. For the higher-level "why" behind these connections, see [Architecture](architecture.md); for per-field message layouts, see the [`autobot_msgs`](../packages/interface/autobot_msgs.md) page directly.

## Command topics

| Topic                    | Type                   | Published by                                                                                                                                                           | Subscribed by                       |
| ------------------------ | ---------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------- |
| `abv/guidance/command`   | `AbvGuidanceCommand`   | `autobot_gui` (pose goals as `line`/`avoid`, paths), `autobot_commander` (`path`), `autobot_bridge`                                                                    | `autobot_guidance`                  |
| `abv/controller/command` | `AbvControllerCommand` | `autobot_guidance` (waypoints, `STOP`), `autobot_commander`, `autobot_teleop`, `autobot_gui` (thruster/direction/velocity/`STOP`), `autobot_policy_registry` (`clear`) | `autobot_goal_manager`              |
| `abv/controller/context` | `AbvControlContext`    | `autobot_goal_manager` (each tick in pose/velocity control)                                                                                                            | `autobot_control`, `autobot_rl-cpp` |
| `abv/thrusters/command`  | `AbvThrusterCommand`   | `autobot_goal_manager` (direct, untagged), `autobot_control` (`control`), `autobot_rl-cpp` (`rl`)                                                                      | `autobot_thrusters`                 |

## Control policy topics

| Topic                 | Type                | Published by                                           | Subscribed by                                                           |
| --------------------- | ------------------- | ------------------------------------------------------ | ----------------------------------------------------------------------- |
| `abv/policy/status`   | `AbvHeartbeat`      | `autobot_control` (`control`), `autobot_rl-cpp` (`rl`) | `autobot_policy_registry`                                               |
| `abv/policy/active`   | `AbvPolicyActive`   | `autobot_policy_registry`                              | `autobot_control`, `autobot_rl-cpp`, `autobot_thrusters`, `autobot_gui` |
| `abv/policy/registry` | `AbvPolicyRegistry` | `autobot_policy_registry`                              | `autobot_gui`                                                           |

## State and status topics

| Topic                   | Type                  | Published by                                                                                                                                          | Subscribed by                                                                                    |
| ----------------------- | --------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------ |
| `abv/state`             | `AbvState`            | `autobot_navigation`                                                                                                                                  | `autobot_guidance`, `autobot_goal_manager`, `autobot_thrusters`, `autobot_gui`, `autobot_bridge` |
| `abv/sim/state`         | `AbvState`            | `autobot_simulator` (when `Navigation.Interface` isn't `optitrack`)                                                                                   | `autobot_navigation` (`simulated` interface)                                                     |
| `abv/sim/gazebo_state`  | `AbvState`            | `autobot_bridge` (`GazeboStateConvertor`)                                                                                                             | `autobot_simulator` (when `Control.Dynamics.PropagationMode: external`)                          |
| `abv/controller/status` | `AbvControllerStatus` | `autobot_goal_manager`                                                                                                                                | `autobot_guidance`, `autobot_gui`, `autobot_bridge`                                              |
| `abv/thrusters/status`  | `AbvThrusterStatus`   | `autobot_thrusters`                                                                                                                                   | `autobot_navigation`, `autobot_gui`                                                              |
| `abv/guidance/status`   | `AbvGuidanceStatus`   | `autobot_guidance`                                                                                                                                    | `autobot_gui`                                                                                    |
| `abv/guidance/path`     | `AbvPath`             | `autobot_guidance`                                                                                                                                    | `autobot_gui`                                                                                    |
| `abv/scene/obstacles`   | `AbvObstacleArray`    | `autobot_gui`                                                                                                                                         | `autobot_guidance`                                                                               |
| `abv/heartbeat`         | `AbvHeartbeat`        | `autobot_goal_manager`, `autobot_control`, `autobot_thrusters`, `autobot_policy_registry`, `autobot_navigation`, `autobot_guidance`, `autobot_bridge` | `autobot_gui`                                                                                    |

## External integration topics (via `autobot_bridge`)

| Topic                             | Type                          | Direction                                                                                                          |
| --------------------------------- | ----------------------------- | ------------------------------------------------------------------------------------------------------------------ |
| `robot/state`                     | `ptera_msgs/RobotState`       | `abv/state` → external autonomy stack                                                                              |
| `robot/vehicle/controller_status` | `ptera_msgs/ControllerStatus` | `abv/controller/status` → external autonomy stack                                                                  |
| `robot/vehicle/waypoint`          | `ptera_msgs/VehicleWaypoint`  | external autonomy stack → `abv/guidance/command`                                                                   |
| `mujoco/robot/state`              | `ptera_msgs/RobotState`       | external simulator → `abv/sim/gazebo_state`                                                                        |
| `mujoco/wrench_cmd`               | `geometry_msgs/WrenchStamped` | `autobot_simulator` → external simulator's `WrenchApplicator` (topic set by `Control.Dynamics.WrenchCommandTopic`) |

## Non-ROS links

| Link                 | From                                                   | To                  |
| -------------------- | ------------------------------------------------------ | ------------------- |
| UDP `127.0.0.1:6969` | `autobot_thrusters` (`UdpThrusterDriver`, x86 builds)  | `autobot_simulator` |
| JETGPIO              | `autobot_thrusters` (`GpioThrusterDriver`, ARM builds) | thruster solenoids  |

## Services

| Service             | Type              | Called by                                                     | Served by                 |
| ------------------- | ----------------- | ------------------------------------------------------------- | ------------------------- |
| `abv/policy/switch` | `AbvPolicySwitch` | `autobot_gui` (`PolicySelectorPanel`), or `ros2 service call` | `autobot_policy_registry` |

## Reading the pipeline end to end

A single pose command, from click to thrust, touches the stack in this order:

1. An operator click-drags a goal on `autobot_gui`'s table-top view. The GUI publishes it on `abv/guidance/command` as type `line`, or `avoid` if any obstacles are placed.
2. `autobot_guidance` generates the path (one waypoint for `line`; a re-planned A\* path for `avoid`) and sends the next waypoint on `abv/controller/command`, publishing the remaining path on `abv/guidance/path` for the GUI to draw.
3. `autobot_goal_manager` enters `POSE_CONTROL` and, every tick, publishes the current state, goal, and error on `abv/controller/context`.
4. The **active** control policy (`autobot_control` or `autobot_rl-cpp`, chosen via `autobot_policy_registry`) computes a control input and publishes it on `abv/thrusters/command`, tagged with its policy name.
5. `autobot_thrusters` drops it unless it matches `abv/policy/active`, otherwise rotates it into the body frame, allocates it to thrusters, and fires them over GPIO (or UDP to `autobot_simulator`). It publishes the pattern and applied force/torque on `abv/thrusters/status`.
6. `autobot_navigation` uses that applied force/torque as the EKF control input, folds in the next measurement (OptiTrack, or `abv/sim/state` in simulation), and publishes an updated `abv/state`.
7. Once the error stays within tolerance for `Control.Arrival.Duration`, `autobot_goal_manager` reports `ARRIVED` on `abv/controller/status`, and `autobot_guidance` sends the next waypoint or goes idle.
