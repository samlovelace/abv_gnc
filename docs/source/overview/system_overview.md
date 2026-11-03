# System Overview

The Air-Bearing Vehicles (ABVs) are a pair of 3-DOF (x, y, yaw) spacecraft simulators owned and operated by [The Autonomy Lab](https://www.theautonomylab.com/) at Florida Tech. They are used to run guidance, navigation, control, and autonomy experiments — formation flying, rendezvous and proximity operations, and contact dynamics — in a way that is representative of free-floating spacecraft dynamics without needing to fly an actual satellite.

Each vehicle floats on a set of air bearings, fed by an on-board nitrogen tank, that lift it a fraction of a millimeter off a flat glass table. This near-frictionless contact lets the vehicle move as if it were unconstrained in the plane, which is what makes the platform a useful stand-in for orbital dynamics despite being confined to a tabletop. A second on-board nitrogen tank pressurizes 8 cold-gas thrusters, arranged to independently command translation (x, y) and rotation (yaw); these are the ABV's only means of actuation, just as thrusters are for a real spacecraft.

## What the software stack does

`autobot` is the software that runs on and around the vehicle to close the guidance, navigation, and control loop:

- **Sense** the vehicle's current pose and velocity (`autobot_navigation`), either from an OptiTrack motion-capture system or, in simulation, from `autobot_simulator`.
- **Decide** where the vehicle should go next (`autobot_guidance`), from a single commanded pose, a sequence of waypoints loaded from a file, or a collision-avoiding path planned around obstacles.
- **Act** by tracking the goal (`autobot_goal_manager`), computing a control input with whichever control policy is active — the on-board PID controller (`autobot_control`) or an RL policy (`autobot_rl-cpp`), switchable at runtime via `autobot_policy_registry` — and converting it into individual thruster firings (`autobot_thrusters`).
- **Operate** the vehicle from a ground station, via a desktop GUI (`autobot_gui`), a command-line tool (`autobot_commander`), or direct keyboard/controller teleop (`autobot_teleop`).

See [Architecture](architecture.md) for how these pieces fit together, and [Data Flow](data_flow.md) for the topic-level view of what talks to what.

## Where it runs

The GNC nodes (`autobot_guidance`, `autobot_navigation`, `autobot_goal_manager`, `autobot_control`, `autobot_rl-cpp`, `autobot_policy_registry`, `autobot_thrusters`) run on an NVIDIA Jetson Orin carried on board the vehicle, since actuating the thrusters over GPIO requires direct hardware access. An operator laptop, on the same network, runs the ground-station tooling (`autobot_gui`, `autobot_commander`, `autobot_teleop`) to monitor and command the vehicle during an experiment. See [Installation](../getting_started/installation.md) and [Operation](../operation/index.md) for how to set up and run both sides.
