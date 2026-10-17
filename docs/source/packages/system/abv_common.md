# abv_common

Shared C++ library used by every other node in the stack.

---

## Purpose

The `abv_common` package is a library, not a node — it has no executable and publishes/subscribes to nothing on its own. It exists so that concerns every C++ node needs (talking to ROS, reading configuration, logging, liveness, low-level networking) are implemented once and linked in everywhere, instead of duplicated per package.

---

## Architecture

Core classes:

- `RosTopicManager` — a singleton wrapper around a single `rclcpp::Node`. `getInstance("name")` creates/returns the node (a node name is only needed on the first call); `createPublisher<T>()` / `publishMessage<T>()` and `createSubscriber<T>()` provide simple templated pub/sub without every package writing its own `rclcpp::Node` subclass. Every node in the stack (`abv_navigation`, `abv_guidance`, `abv_goal_manager`, `abv_thrusters`, `abv_bridge`, `abv_gui`, ...) is, under the hood, this one class.
- `ConfigurationManager` — a singleton that loads and parses `abv_bringup`'s `config.yaml` once at startup (`loadConfiguration()`), and hands out typed sub-configs (`GuidanceConfig`, `NavigationConfig`, `ControlConfig`, `ControlLawConfig`, `ThrusterConfig`, `HeartbeatConfig`, `TableViewConfig`, `SceneConfig`, `CollisionAvoidConfig`) via getters. Every node calls this before doing anything else.
- `HeartbeatPublisher` — constructed once with a node name (e.g. `"goal_manager"`) and kept alive for the node's lifetime; publishes `AbvHeartbeat` at `Heartbeat.Rate`. The topic defaults to `abv/heartbeat` (which `abv_gui`'s Node Health panel relies on); control-policy nodes construct a second one on `abv/policy/status` to register with `abv_policy_registry`.
- `DataLogger` — a singleton for writing timestamped log/data files under a per-run directory (`createMainLog()` once per process, then `createLog()` per named file and `write()` to append lines or numeric vectors).
- `Watchdog` — a small reusable one-shot timer: `start(duration, callback)` fires `callback` on a background thread after `duration` seconds unless `cancel()` is called first. Used where a component needs to detect "nothing happened in time" (e.g. `RosNavigationListener`'s nav-data freshness check). `abv_guidance` uses its own `PathWatchdog` for path/waypoint timeouts.
- `UdpClient` — a minimal cross-platform (POSIX/Windows) UDP socket wrapper used for out-of-band communication that doesn't go through ROS (e.g. `abv_thrusters`' `UdpThrusterDriver`, which sends firing patterns to `abv_simulator`).
- `RosNavigationListener` — a small helper for subscribing to `abv/state` without hand-rolling the subscription in each consumer. Also tracks freshness: `isNavOk()` is true only if a valid `AbvState` arrived within `Control.NavDataTimeout` seconds.
- Smaller utilities: `RateController` (fixed-rate loops), `ThreadSafe<T>` (mutex-guarded value), `ConsumableBuffer`, `SignalHandler` (SIGINT shutdown), and `ArrivalStatus` (`IDLE`/`RUNNING`/`ARRIVED`).

## Topics

- `/abv/heartbeat`, `/abv/policy/status` — published by `HeartbeatPublisher` on behalf of whichever node constructed it.
- `/abv/state` — subscribed by `RosNavigationListener` on behalf of its owner. `abv_common` itself is not a node and publishes nothing directly.
