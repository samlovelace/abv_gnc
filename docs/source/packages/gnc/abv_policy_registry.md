# abv_policy_registry

Runtime discovery and switching of control policies for the Air Bearing Vehicle (ABV) platform.

---

## Purpose

More than one control-policy node can run at once (`abv_control` as `control`, `abv_rl-cpp` as `rl`). The `abv_policy_registry` package tracks which of them are alive and holds the single source of truth for which one is allowed to drive the thrusters.

**It starts with no active policy.** Until an operator switches to one — from `abv_gui`'s policy selector or by calling `abv/policy/switch` — `abv_thrusters` drops every policy-sourced command, so pose and velocity goals do nothing. Direct thruster/direction commands (teleop, the GUI's thruster buttons) are not affected.

---

## How It Works

- **Discovery** — each policy node publishes an `AbvHeartbeat` on `abv/policy/status`. Every `node_name` seen there is registered as a policy. A policy is marked stale if no heartbeat arrives within `Heartbeat.StaleAfter` seconds.
- **Broadcast** — at `Heartbeat.Rate`, the registry re-publishes the active policy on `abv/policy/active` and the full list of known policies (with `alive` flags) on `abv/policy/registry`.
- **Switching** — the `abv/policy/switch` service ([`AbvPolicySwitch`](../interface/abv_msgs.md#abvpolicyswitch)) is rejected if the requested policy isn't registered or is stale. On a successful switch, the registry publishes the new active policy and sends `AbvControllerCommand{type: "clear"}` on `abv/controller/command`, so `abv_goal_manager` drops the current goal instead of continuing it under the new policy.

To switch from the command line:

```bash
ros2 service call /abv/policy/switch abv_msgs/srv/AbvPolicySwitch "{policy_name: control}"
```

---

## Topics and Services

### Subscribed

- `/abv/policy/status` — policy heartbeats

### Published

- `/abv/policy/active` — [`AbvPolicyActive`](../interface/abv_msgs.md#abvpolicyactive)
- `/abv/policy/registry` — [`AbvPolicyRegistry`](../interface/abv_msgs.md#abvpolicyregistry--abvpolicyentry)
- `/abv/controller/command` — `clear` after a switch
- `/abv/heartbeat` — liveness, as `policy_registry`

### Services

- `/abv/policy/switch` — [`AbvPolicySwitch`](../interface/abv_msgs.md#abvpolicyswitch)
