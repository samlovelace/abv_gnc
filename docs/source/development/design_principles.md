# Software Design Principles

This document describes the core software design principles used throughout the Air Bearing Vehicle (ABV) software stack.

The purpose of this page is to explain architectural decisions and provide guidance (pun intended) for future development. These principles help maintain consistency, safety, and modularity across the system.

---

## Modular Architecture

The ABV stack is structured as independent ROS 2 packages with clearly defined responsibilities.

Examples include:

- `abv_navigation` — State estimation
- `abv_guidance` — Trajectory and waypoint generation
- `abv_control` — Feedback control and thruster command computation
- `abv_msgs` — Interface definitions
- Tooling packages (teleoperation, visualization, bringup)

Each package owns a single primary responsibility.

**Rationale:**

- Reduces coupling between subsystems
- Simplifies debugging and testing
- Enables module replacement without rewriting the entire stack
- Supports both simulation and hardware execution

---

## Clear Interface Boundaries

All communication between packages occurs through ROS 2 topics using message definitions from `abv_msgs`.

Packages do not directly depend on internal classes from other packages.

**Rationale:**

- Enforces abstraction boundaries
- Supports distributed execution
- Enables runtime module substitution
- Prevents tight coupling between subsystems

---

## Separation of Algorithm and Hardware

Algorithmic logic (control laws, estimation filters, planning algorithms) is separated from hardware-specific implementations (GPIO drivers, UDP interfaces, motion capture adapters).

Hardware access is isolated behind well-defined interfaces.

**Rationale:**

- Enables simulation-first development
- Allows hardware replacement without modifying algorithms
- Improves portability (Jetson vs x86)
- Reduces risk during experimentation

---

## Configuration Over Hardcoding

Behavior should be configurable through:

- YAML configuration files
- ROS 2 parameters
- Launch configurations

Hardcoded constants should be avoided unless fundamental to system definition.

**Rationale:**

- Enables rapid experimentation
- Simplifies parameter tuning
- Improves reproducibility of experiments

---

## Simulation-First Development

New functionality should be validated in simulation before deployment on hardware.

The stack supports simulated state inputs and command pathways to enable safe testing without activating physical thrusters.

**Rationale:**

- Prevents hardware damage
- Speeds up development iteration
- Enables reproducible experiments
- Encourages safe experimentation practices

---

## Explicit Data Contracts

All message definitions must:

- Clearly specify units
- Clearly specify coordinate frame conventions
- Avoid ambiguous semantics
- Maintain backward compatibility where possible

**Rationale:**

Robotics systems frequently fail due to unit or frame inconsistencies.  
Clear and explicit data contracts reduce integration errors and debugging time.

---

## Deterministic Control Loops

Control-related components should:

- Operate at a clearly defined rate
- Avoid blocking calls in control paths
- Avoid unpredictable dynamic memory allocation in critical loops

**Rationale:**

Predictable timing behavior is essential for stable control performance.

---

## Extensibility

The architecture is designed so that:

- Alternative controllers can replace existing ones
- Different state estimators can be inserted
- Additional sensors can be integrated
- New tools can subscribe to existing topics

without requiring structural changes to core modules.

**Rationale:**

The ABV stack is intended as a research platform. Flexibility and extensibility are essential.

---

## Safety Awareness

Because the ABV stack controls physical hardware:

- Control saturation limits are enforced
- Simulation validation is encouraged
- Hardware interfaces are isolated from algorithmic code

Safety considerations influence architectural decisions at all levels.

---

## Ongoing Evolution

This document should evolve as the architecture matures.

When significant architectural decisions are made, they should be documented here to preserve design intent and guide future development.
