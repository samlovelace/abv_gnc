# abv_rl / abv_rl-cpp

Experimental reinforcement-learning control policies for the Air Bearing Vehicle (ABV) platform.

---

## abv_rl-cpp

The C++ RL policy node, launched by both `gnc.launch.py` and `jetson.launch.py`. It registers with [`abv_policy_registry`](abv_policy_registry.md) as the `rl` policy and, when active, plays the same role as [`abv_control`](abv_control.md): it turns each `AbvControlContext` from `abv_goal_manager` into a control input.

- Runs the trained protagonist network with ONNX Runtime. Models (`protagonist.onnx`, `adversary.onnx`, `critic_<i>.onnx`) live in `abv_rl-cpp/abv_rl_onnx/`; only the protagonist is evaluated at runtime.
- Observation: goal-relative x/y error, yaw, body velocities, and per-obstacle direction/margin terms. The obstacle set is currently **hard-coded** in the node (3 obstacles), not read from `abv/scene/obstacles`.
- Output: a **body-frame** `direction` command on `abv/thrusters/command`, tagged `policy_name: rl`.

### Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `model_dir` | developer-machine path to `abv_rl_onnx/` | directory containing the `.onnx` models — override this on other machines |
| `num_obstacles` | `3` | obstacle terms in the observation |
| `num_critics` | `3` | critic models to load |

### Build requirements

ONNX Runtime must be installed; CMake stops with an error if it isn't found. Point it at a non-system install with `-DORT_ROOT=<path>` (expects `include/onnxruntime/` and `lib/libonnxruntime.so`). `scripts/setup.sh` does **not** install ONNX Runtime. On the Jetson, `jetson.launch.py` also adds CUDA 12.6 and the ONNX Runtime install to `LD_LIBRARY_PATH`.

### Topics

- Subscribed: `/abv/controller/context`, `/abv/policy/active`
- Published: `/abv/thrusters/command`, `/abv/policy/status` (as `rl`)

---

## abv_rl

The original Python package: SAC training code (`SAC.py`, `model.py`), the PyTorch checkpoints in `abv_rl/best_models/`, and `export_onnx.py`, which produces the models used by `abv_rl-cpp`. It expects the conda environment `safetyrl-env` (see `abv_rl/environment.yml`; `scripts/abv_rl_venv.sh` activates it and runs the node).

The Python `rl_policy_node.py` still uses the removed `AbvControlAction` service and does not work with the current policy-switching interface. Use `abv_rl-cpp` for runtime control.
