# Data Flow

This page lists every topic and service used across the ABV stack: who publishes/calls it, who subscribes/serves it, and which [`abv_msgs`](../packages/interface/abv_msgs.md) type it carries. For the higher-level "why" behind these connections, see [Architecture](architecture.md); for per-field message layouts, see the [`abv_msgs`](../packages/interface/abv_msgs.md) page directly.

## Command topics

| Topic | Type | Published by | Subscribed by |
|---|---|---|---|
| `abv/controller/command` | `AbvControllerCommand` | `abv_guidance`, `abv_commander`, `abv_teleop`, `abv_gui` | `abv_controller` |
| `abv/guidance/command` | `AbvGuidanceCommand` | `abv_commander`, `abv_gui` | `abv_guidance` |

## State and status topics

| Topic | Type | Published by | Subscribed by |
|---|---|---|---|
| `abv/state` | `AbvState` | `abv_navigation` | `abv_guidance`, `abv_controller`, `abv_gui`, `abv_bridge` |
| `abv/sim/state` | `AbvState` | `abv_simulator` | `abv_navigation` (sim mode) |
| `abv/sim/gazebo_state` | `AbvState` | `abv_bridge` (`GazeboStateConvertor`) | `abv_controller` (when `Dynamics.PropagationMode: external`) |
| `abv/controller/status` | `AbvControllerStatus` | `abv_controller` | `abv_guidance`, `abv_navigation`, `abv_gui`, `abv_bridge` |
| `abv/controller/thrusters` | `AbvThrusterStatus` | `abv_controller` | `abv_gui` |
| `abv/guidance/status` | `AbvGuidanceStatus` | `abv_guidance` | `abv_gui` |
| `abv/heartbeat` | `AbvHeartbeat` | every long-running node (`abv_controller`, `abv_navigation`, `abv_guidance`, `abv_bridge`) | `abv_gui` |

## External integration topics (via `abv_bridge`)

| Topic | Type | Direction |
|---|---|---|
| `robot/state` | `ptera_msgs/RobotState` | `abv/state` → external autonomy stack |
| `robot/vehicle/controller_status` | `ptera_msgs/ControllerStatus` | `abv/controller/status` → external autonomy stack |
| `robot/vehicle/waypoint` | `ptera_msgs/VehicleWaypoint` | external autonomy stack → `abv/guidance/command` |
| `gazebo/robot/state` | `ptera_msgs/RobotState` | Gazebo → `abv/sim/gazebo_state` |
| `gazebo/wrench_cmd` | `geometry_msgs/WrenchStamped` | `abv_controller` → Gazebo's `WrenchApplicator` |

## Services

| Service | Type | Called by | Served by |
|---|---|---|---|
| `abv/control_action` | `AbvControlAction` | `abv_controller` (`ExternalControlPolicy`, when `Control.ControlPolicy: External`) | an external control policy node (e.g. `abv_rl`/`abv_rl-cpp`) |

## Reading the pipeline end to end

A single pose command, from click to thrust, touches the stack in this order:

1. An operator sends a pose via `abv_gui`'s table-top view (or `abv_commander`, or `abv_teleop`) on `abv/controller/command`, or a full path via `abv_guidance` on `abv/guidance/command`.
2. If it went through `abv_guidance`, the guidance state machine forwards the current waypoint to `abv_controller` on `abv/controller/command`, waiting for `abv/controller/status.arrival` to report `ARRIVED` before sending the next one.
3. `abv_controller` computes control effort from the state error between the command and the latest `abv/state`, maps it to thruster firings, and publishes the applied force/torque and arrival state back out on `abv/controller/status` (and, for observability, `abv/controller/thrusters`).
4. `abv_navigation` folds that applied force/torque into its EKF as a control input, and folds in the next raw measurement (OptiTrack, or `abv/sim/state` in simulation) to publish an updated `abv/state` — closing the loop back to step 1/2.
