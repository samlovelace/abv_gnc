# Packages

The ABV software stack is implemented as ROS2 packages with each package handling a primary component, i.e. there is one package for performing guidance, one package for navigation, one package for control and so on.

Each package name begins with the prefix "abv\_" followed by a descriptive word to represent what it does. Packages are grouped below by role: [GNC](gnc/index.md) (the on-board sense/decide/act pipeline), [Interfaces](interface/index.md) (shared message definitions and external integration), [Tools](tools/index.md) (operator/ground-station software), and [System](system/index.md) (launch, configuration, and shared library code).

| Package                                                     | Group     | Description                                               |
| ----------------------------------------------------------- | --------- | --------------------------------------------------------- |
| [`autobot_navigation`](gnc/autobot_navigation.md)           | GNC       | State estimation via EKF, OptiTrack integration           |
| [`autobot_guidance`](gnc/autobot_guidance.md)               | GNC       | Waypoint sequencing and path following                    |
| [`autobot_goal_manager`](gnc/autobot_goal_manager.md)       | GNC       | Command intake, goal state machine, arrival detection     |
| [`autobot_control`](gnc/autobot_control.md)                 | GNC       | Classical control laws (PID) — the `control` policy       |
| [`autobot_thrusters`](gnc/autobot_thrusters.md)             | GNC       | Thruster allocation, policy arbitration, GPIO/UDP drivers |
| [`autobot_policy_registry`](gnc/autobot_policy_registry.md) | GNC       | Runtime discovery and switching of control policies       |
| [`autobot_rl-cpp`](gnc/autobot_rl.md)                       | GNC       | Experimental RL control policy (ONNX) — the `rl` policy   |
| [`autobot_rl`](gnc/autobot_rl.md#autobot_rl)                | GNC       | RL training code and ONNX export (Python)                 |
| [`autobot_msgs`](interface/autobot_msgs.md)                 | Interface | Custom ROS2 message and service definitions               |
| [`autobot_bridge`](interface/autobot_bridge.md)             | Interface | Bridge between `autobot_msgs` and external interfaces     |
| [`autobot_gui`](tools/autobot_gui.md)                       | Tools     | Desktop GUI for live telemetry and sending commands       |
| [`autobot_commander`](tools/autobot_commander.md)           | Tools     | Command-line tool for sending commands                    |
| [`autobot_simulator`](tools/autobot_simulator.md)           | Tools     | Numerical simulator for ABV dynamics                      |
| [`autobot_teleop`](tools/autobot_teleop.md)                 | Tools     | Keyboard or game controller teleop of the ABV             |
| [`autobot_bringup`](system/autobot_bringup.md)              | System    | Launch files and configuration                            |
| [`autobot_common`](system/autobot_common.md)                | System    | Shared C++ library used by every other package            |
| [`autobot_description`](system/autobot_description.md)      | System    | URDF description with RViz launch files                   |

```{toctree}
:maxdepth: 1

gnc/index
interface/index
tools/index
system/index
```
