# TODO Tasks for documentation

## Done (2026-08-30)

- Installation page now lists the expected `abv_*` packages under "Verify Installation".
- Added a Quick Start / Operation section (`docs/source/operation/index.md`) covering Jetson/laptop setup checks, starting the GNC stack, starting `abv_gui`, checking comms via the Node Health panel, and sending pose commands from the GUI, with a pointer to the full `abv_gui` page.
- Added missing per-package docs: `abv_gui`, `abv_bridge`, `abv_bringup`, `abv_common`, `abv_description`.
- Added an Overview section covering the ABV's purpose (`system_overview.md`) and a system diagram (`architecture.md`) showing the on-board GNC stack and both command sources — a human operator via `abv_gui`, and an autonomy module via `abv_bridge`.
- Fixed pre-existing issues found along the way: broken toctree references (`operation/index`, `overview/data_flow` — both now real pages), the `abv_control`/`abv_controller` naming bug, a duplicated sentence in `abv_guidance.md`, stale/incomplete message field docs and the unfilled `AbvResponse` template in `abv_msgs.md`, and thin `abv_navigation.md` architecture/topic coverage.

## Still open

- `abv_rl` and `abv_rl-cpp` (experimental RL control policy packages) have no docs page yet — deferred since they're not yet in the README's package table either.
- Root `readme.md`'s Quick Start section duplicates (doesn't conflict with, but overlaps) the new `docs/source/operation/index.md` — worth deciding whether to trim the README to a pointer at some point.

## Other

Documentation should be digestible yet thorough. Open to suggestions on other documentation that could be beneficial.
