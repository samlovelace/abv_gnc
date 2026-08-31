# abv_common

Shared C++ library used by every other node in the stack.

---

## Purpose

The `abv_common` package is a library, not a node — it has no executable and publishes/subscribes to nothing on its own. It exists so that concerns every C++ node needs (talking to ROS, reading configuration, logging, liveness, low-level networking) are implemented once and linked in everywhere, instead of duplicated per package.

---

## Architecture

Core classes:

- `RosTopicManager` — a singleton wrapper around a single `rclcpp::Node`. `getInstance("name")` creates/returns the node (a node name is only needed on the first call); `createPublisher<T>()` / `publishMessage<T>()` and `createSubscriber<T>()` provide simple templated pub/sub without every package writing its own `rclcpp::Node` subclass. Every node in the stack (`abv_navigation`, `abv_guidance`, `abv_controller`, `abv_bridge`, `abv_gui`, ...) is, under the hood, this one class.
- `ConfigurationManager` — a singleton that loads and parses `abv_bringup`'s `config.yaml` once at startup (`loadConfiguration()`), and hands out typed sub-configs (`GuidanceConfig`, `NavigationConfig`, `ControlConfig`, `HeartbeatConfig`, `TableViewConfig`) via getters. Every node calls this before doing anything else.
- `HeartbeatPublisher` — constructed once with a node name (e.g. `"controller"`) and kept alive for the node's lifetime; publishes `AbvHeartbeat` on `abv/heartbeat` at `Heartbeat.Rate`, which `abv_gui`'s Node Health panel relies on.
- `DataLogger` — a singleton for writing timestamped log/data files under a per-run directory (`createMainLog()` once per process, then `createLog()` per named file and `write()` to append lines or numeric vectors).
- `Watchdog` — a small reusable one-shot timer: `start(duration, callback)` fires `callback` on a background thread after `duration` seconds unless `cancel()` is called first. Used where a component needs to detect "nothing happened in time" (e.g. `abv_guidance`'s waypoint timeout).
- `UdpClient` — a minimal cross-platform (POSIX/Windows) UDP socket wrapper used for out-of-band communication that doesn't go through ROS (e.g. `abv_simulator`'s thruster-command listener).
- `RosNavigationListener` — a small helper for subscribing to `abv/state` without hand-rolling the subscription in each consumer.

## Topics

- `/abv/heartbeat` — published by `HeartbeatPublisher` on behalf of whichever node constructed it. `abv_common` itself is not a node and publishes nothing directly.
