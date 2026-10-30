# Packages

The ABV software stack is implemented as ROS2 packages with each package handling a primary component, i.e. there is one package for performing guidance, one package for navigation, one package for control and so on.

Each package name begins with the prefix "abv\_" followed by a descriptive word to represent what it does. Packages are grouped below by role: [GNC](gnc/index.md) (the on-board sense/decide/act pipeline), [Interfaces](interface/index.md) (shared message definitions and external integration), [Tools](tools/index.md) (operator/ground-station software), and [System](system/index.md) (launch, configuration, and shared library code).

| Package | Group | Description |
|---|---|---|
| [`abv_navigation`](gnc/abv_navigation.md) | GNC | State estimation via EKF, OptiTrack integration |
| [`abv_guidance`](gnc/abv_guidance.md) | GNC | Waypoint sequencing and path following |
| [`abv_goal_manager`](gnc/abv_goal_manager.md) | GNC | Command intake, goal state machine, arrival detection |
| [`abv_control`](gnc/abv_control.md) | GNC | Classical control laws (PID) — the `control` policy |
| [`abv_thrusters`](gnc/abv_thrusters.md) | GNC | Thruster allocation, policy arbitration, GPIO/UDP drivers |
| [`abv_policy_registry`](gnc/abv_policy_registry.md) | GNC | Runtime discovery and switching of control policies |
| [`abv_rl-cpp`](gnc/abv_rl.md) | GNC | Experimental RL control policy (ONNX) — the `rl` policy |
| [`abv_rl`](gnc/abv_rl.md#abv_rl) | GNC | RL training code and ONNX export (Python) |
| [`abv_msgs`](interface/abv_msgs.md) | Interface | Custom ROS2 message and service definitions |
| [`abv_bridge`](interface/abv_bridge.md) | Interface | Bridge between `abv_msgs` and external interfaces |
| [`abv_gui`](tools/abv_gui.md) | Tools | Desktop GUI for live telemetry and sending commands |
| [`abv_commander`](tools/abv_commander.md) | Tools | Command-line tool for sending commands |
| [`abv_simulator`](tools/abv_simulator.md) | Tools | Numerical simulator for ABV dynamics |
| [`abv_teleop`](tools/abv_teleop.md) | Tools | Keyboard or game controller teleop of the ABV |
| [`abv_bringup`](system/abv_bringup.md) | System | Launch files and configuration |
| [`abv_common`](system/abv_common.md) | System | Shared C++ library used by every other package |
| [`abv_description`](system/abv_description.md) | System | URDF description with RViz launch files |

```{toctree}
:maxdepth: 1

gnc/index
interface/index
tools/index
system/index
```
