# Data Flow

This page lists every topic and service used across the ABV stack: who publishes/calls it, who subscribes/serves it, and which [`abv_msgs`](../packages/interface/abv_msgs.md) type it carries. For the higher-level "why" behind these connections, see [Architecture](architecture.md); for per-field message layouts, see the [`abv_msgs`](../packages/interface/abv_msgs.md) page directly.

## Command topics

| Topic | Type | Published by | Subscribed by |
|---|---|---|---|
| `abv/guidance/command` | `AbvGuidanceCommand` | `abv_gui` (pose goals as `line`/`avoid`, paths), `abv_commander` (`path`), `abv_bridge` | `abv_guidance` |
| `abv/controller/command` | `AbvControllerCommand` | `abv_guidance` (waypoints, `STOP`), `abv_commander`, `abv_teleop`, `abv_gui` (thruster/direction/velocity/`STOP`), `abv_policy_registry` (`clear`) | `abv_goal_manager` |
| `abv/controller/context` | `AbvControlContext` | `abv_goal_manager` (each tick in pose/velocity control) | `abv_control`, `abv_rl-cpp` |
| `abv/thrusters/command` | `AbvThrusterCommand` | `abv_goal_manager` (direct, untagged), `abv_control` (`control`), `abv_rl-cpp` (`rl`) | `abv_thrusters` |

## Control policy topics

| Topic | Type | Published by | Subscribed by |
|---|---|---|---|
| `abv/policy/status` | `AbvHeartbeat` | `abv_control` (`control`), `abv_rl-cpp` (`rl`) | `abv_policy_registry` |
| `abv/policy/active` | `AbvPolicyActive` | `abv_policy_registry` | `abv_control`, `abv_rl-cpp`, `abv_thrusters`, `abv_gui` |
| `abv/policy/registry` | `AbvPolicyRegistry` | `abv_policy_registry` | `abv_gui` |

## State and status topics

| Topic | Type | Published by | Subscribed by |
|---|---|---|---|
| `abv/state` | `AbvState` | `abv_navigation` | `abv_guidance`, `abv_goal_manager`, `abv_thrusters`, `abv_gui`, `abv_bridge` |
| `abv/sim/state` | `AbvState` | `abv_simulator` (when `Navigation.Interface` isn't `optitrack`) | `abv_navigation` (`simulated` interface) |
| `abv/sim/gazebo_state` | `AbvState` | `abv_bridge` (`GazeboStateConvertor`) | `abv_simulator` (when `Control.Dynamics.PropagationMode: external`) |
| `abv/controller/status` | `AbvControllerStatus` | `abv_goal_manager` | `abv_guidance`, `abv_gui`, `abv_bridge` |
| `abv/thrusters/status` | `AbvThrusterStatus` | `abv_thrusters` | `abv_navigation`, `abv_gui` |
| `abv/guidance/status` | `AbvGuidanceStatus` | `abv_guidance` | `abv_gui` |
| `abv/guidance/path` | `AbvPath` | `abv_guidance` | `abv_gui` |
| `abv/scene/obstacles` | `AbvObstacleArray` | `abv_gui` | `abv_guidance` |
| `abv/heartbeat` | `AbvHeartbeat` | `abv_goal_manager`, `abv_control`, `abv_thrusters`, `abv_policy_registry`, `abv_navigation`, `abv_guidance`, `abv_bridge` | `abv_gui` |

## External integration topics (via `abv_bridge`)

| Topic | Type | Direction |
|---|---|---|
| `robot/state` | `ptera_msgs/RobotState` | `abv/state` → external autonomy stack |
| `robot/vehicle/controller_status` | `ptera_msgs/ControllerStatus` | `abv/controller/status` → external autonomy stack |
| `robot/vehicle/waypoint` | `ptera_msgs/VehicleWaypoint` | external autonomy stack → `abv/guidance/command` |
| `mujoco/robot/state` | `ptera_msgs/RobotState` | external simulator → `abv/sim/gazebo_state` |
| `mujoco/wrench_cmd` | `geometry_msgs/WrenchStamped` | `abv_simulator` → external simulator's `WrenchApplicator` (topic set by `Control.Dynamics.WrenchCommandTopic`) |

## Non-ROS links

| Link | From | To |
|---|---|---|
| UDP `127.0.0.1:6969` | `abv_thrusters` (`UdpThrusterDriver`, x86 builds) | `abv_simulator` |
| JETGPIO | `abv_thrusters` (`GpioThrusterDriver`, ARM builds) | thruster solenoids |

## Services

| Service | Type | Called by | Served by |
|---|---|---|---|
| `abv/policy/switch` | `AbvPolicySwitch` | `abv_gui` (`PolicySelectorPanel`), or `ros2 service call` | `abv_policy_registry` |

## Reading the pipeline end to end

A single pose command, from click to thrust, touches the stack in this order:

1. An operator click-drags a goal on `abv_gui`'s table-top view. The GUI publishes it on `abv/guidance/command` as type `line`, or `avoid` if any obstacles are placed.
2. `abv_guidance` generates the path (one waypoint for `line`; a re-planned A* path for `avoid`) and sends the next waypoint on `abv/controller/command`, publishing the remaining path on `abv/guidance/path` for the GUI to draw.
3. `abv_goal_manager` enters `POSE_CONTROL` and, every tick, publishes the current state, goal, and error on `abv/controller/context`.
4. The **active** control policy (`abv_control` or `abv_rl-cpp`, chosen via `abv_policy_registry`) computes a control input and publishes it on `abv/thrusters/command`, tagged with its policy name.
5. `abv_thrusters` drops it unless it matches `abv/policy/active`, otherwise rotates it into the body frame, allocates it to thrusters, and fires them over GPIO (or UDP to `abv_simulator`). It publishes the pattern and applied force/torque on `abv/thrusters/status`.
6. `abv_navigation` uses that applied force/torque as the EKF control input, folds in the next measurement (OptiTrack, or `abv/sim/state` in simulation), and publishes an updated `abv/state`.
7. Once the error stays within tolerance for `Control.Arrival.Duration`, `abv_goal_manager` reports `ARRIVED` on `abv/controller/status`, and `abv_guidance` sends the next waypoint or goes idle.
