# TODO Tasks for documentation

## Done (2026-10-16)

- Aligned docs with the `abv_controller` split: new pages for `abv_goal_manager`, `abv_control`, `abv_thrusters`, `abv_policy_registry`, and `abv_rl` / `abv_rl-cpp`; `abv_controller.md` is now a short "removed, see …" redirect.
- Documented runtime control-policy switching (`abv/policy/*`, `AbvPolicySwitch`) and that no policy is active at startup, across Quick Start, Running, README, and the GUI page.
- Documented `abv_guidance`'s `avoid` (A*) mode, including the constant-heading rule, plus `Scene`, `abv/scene/obstacles`, `abv/guidance/path`, the `# PreviewLength` header, and relaxed-tolerance waypoint timeouts.
- Rewrote `data_flow.md`, the `abv_msgs` reference (8 new messages, `AbvPolicySwitch`, removed `AbvControlAction`), and the `abv_bringup` config reference (`ControlLaw`, `Thrusters`, `Scene`, `CollisionAvoid` sections; new keys).
- Replaced the outdated GUI screenshot with two new captures (`abv_gui_no_policy.png`, `abv_gui_policy_with_goal.png`), used in Quick Start and the `abv_gui` page.
- Updated the installation page for the current `setup.sh`, ONNX Runtime, and the `docker/` helpers; regenerated `_static/architecture.png` from the updated `architecture.puml`.

## Done (2026-08-30)

- Installation page now lists the expected `abv_*` packages under "Verify Installation".
- Added a Quick Start / Operation section (`docs/source/operation/index.md`) covering Jetson/laptop setup checks, starting the GNC stack, starting `abv_gui`, checking comms via the Node Health panel, and sending pose commands from the GUI, with a pointer to the full `abv_gui` page.
- Added missing per-package docs: `abv_gui`, `abv_bridge`, `abv_bringup`, `abv_common`, `abv_description`.
- Added an Overview section covering the ABV's purpose (`system_overview.md`) and a system diagram (`architecture.md`) showing the on-board GNC stack and both command sources — a human operator via `abv_gui`, and an autonomy module via `abv_bridge`.
- Fixed pre-existing issues found along the way: broken toctree references (`operation/index`, `overview/data_flow` — both now real pages), a docs-side `abv_control`/`abv_controller` naming mix-up (since superseded — `abv_control` is now a real package), a duplicated sentence in `abv_guidance.md`, stale/incomplete message field docs and the unfilled `AbvResponse` template in `abv_msgs.md`, and thin `abv_navigation.md` architecture/topic coverage.

## Still open

- Root `readme.md`'s Quick Start section duplicates (doesn't conflict with, but overlaps) `docs/source/operation/index.md` — worth deciding whether to trim the README to a pointer at some point.

## Other

Documentation should be digestible yet thorough. Open to suggestions on other documentation that could be beneficial.
