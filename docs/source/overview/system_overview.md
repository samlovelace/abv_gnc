# System Overview

The Air-Bearing Vehicles (ABVs) are a pair of 3-DOF (x, y, yaw) spacecraft simulators owned and operated by [The Autonomy Lab](https://www.theautonomylab.com/) at Florida Tech. They are used to run guidance, navigation, control, and autonomy experiments — formation flying, rendezvous and proximity operations, and contact dynamics — in a way that is representative of free-floating spacecraft dynamics without needing to fly an actual satellite.

Each vehicle floats on a set of air bearings, fed by an on-board nitrogen tank, that lift it a fraction of a millimeter off a flat glass table. This near-frictionless contact lets the vehicle move as if it were unconstrained in the plane, which is what makes the platform a useful stand-in for orbital dynamics despite being confined to a tabletop. A second on-board nitrogen tank pressurizes 8 cold-gas thrusters, arranged to independently command translation (x, y) and rotation (yaw); these are the ABV's only means of actuation, just as thrusters are for a real spacecraft.

## What the software stack does

`abv_gnc` is the software that runs on and around the vehicle to close the guidance, navigation, and control loop:

- **Sense** the vehicle's current pose and velocity (`abv_navigation`), either from an OptiTrack motion-capture system or, in simulation, from `abv_simulator`.
- **Decide** where the vehicle should go next (`abv_guidance`), from a single commanded pose/velocity or a sequence of waypoints loaded from a file.
- **Act** by converting that intent into individual thruster firings (`abv_controller`), using either an on-board PID controller or an externally-hosted control policy.
- **Operate** the vehicle from a ground station, via a desktop GUI (`abv_gui`), a command-line tool (`abv_commander`), or direct keyboard/controller teleop (`abv_teleop`).

See [Architecture](architecture.md) for how these pieces fit together, and [Data Flow](data_flow.md) for the topic-level view of what talks to what.

## Where it runs

The GNC nodes (`abv_guidance`, `abv_navigation`, `abv_controller`) run on an NVIDIA Jetson Orin carried on board the vehicle, since actuating the thrusters over GPIO requires direct hardware access. An operator laptop, on the same network, runs the ground-station tooling (`abv_gui`, `abv_commander`, `abv_teleop`) to monitor and command the vehicle during an experiment. See [Installation](../getting_started/installation.md) and [Operation](../operation/index.md) for how to set up and run both sides.
