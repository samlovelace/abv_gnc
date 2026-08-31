# abv_description

URDF description and RViz visualization for the ABV chassis.

---

## Purpose

The `abv_description` package provides a URDF/xacro model of the vehicle and a launch file to visualize it in RViz. It contains no nodes and no GNC logic — it exists purely to give the vehicle a visual/kinematic representation, independent of whether the GNC stack is running.

---

## Model

`urdf/abv_chassis.urdf.xacro` models the vehicle's 3-DOF planar motion explicitly as a joint chain rather than a free-floating body, so it can be driven directly from published joint states in RViz:

- `x_joint` (prismatic, x) → `y_joint` (prismatic, y) → `yaw_joint` (revolute, yaw) → `vehicle_link`, which carries the visual mesh (`meshes/abv_chassis/abv_v2.dae`).

`urdf/abv.urdf.xacro` is the top-level model that instantiates the chassis macro; `urdf/abv_chassis-standalone.urdf.xacro` provides a standalone (non-macro) version for previewing the chassis alone. `urdf/manipulators/` additionally holds xacro models for arm attachments (`widow_xl`, `mycobot_280_pi`) used on variants of the platform carrying a manipulator.

## Usage

```bash
ros2 launch abv_description visualize.launch.py
```

This starts `robot_state_publisher` (publishing the xacro-processed URDF as `robot_description`), a static `world` → `base_link` transform, and `rviz2` pre-configured with `rviz/abv.rviz`. It does not require the GNC stack to be running, and publishes no `abv/*` topics itself.
