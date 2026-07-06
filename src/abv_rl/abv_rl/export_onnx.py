"""
export_onnx.py  –  Export trained protagonist / adversary / critics to ONNX.

Usage:
    python export_onnx.py \
        --best_dir /abv_gnc/src/abv_rl/abv_rl/best_models \
        --out_dir  /abv_gnc/src/abv_rl/abv_rl/best_models \
        --num_obstacles 3 \
        --num_critics   3 \
        [--config_yaml  /abv_gnc/src/abv_rl/abv_rl/config.yaml]

What gets exported
------------------
protagonist.onnx  — input: [1, state_dim], output: [1, 3]  (deterministic action)
adversary.onnx    — input: [1, state_dim], output: [1, 3]  (deterministic disturbance)
critic_N.onnx     — input: [1, state_dim+3+3], output: [1, 1], [1, 1]  (q1, q2)

The protagonist / adversary wrappers call .sample() and return only the
deterministic mean (the third return value) so the C++ node gets a clean
[x, y, yaw] tensor without having to understand the Gaussian internals.
"""

import argparse
import os
import sys
import torch
import torch.nn as nn
import yaml
import numpy as np
import gymnasium.spaces
from types import SimpleNamespace
from copy import deepcopy

# --------------------------------------------------------------------------- #
# Make sure the package containing SAC / model / config is importable          #
# --------------------------------------------------------------------------- #
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

from SAC    import SAC
from config import ceConfig


# --------------------------------------------------------------------------- #
# Thin wrappers that give a single clean output tensor                         #
# --------------------------------------------------------------------------- #

# class ProtagonistExport(nn.Module):
#     """Wraps GaussianPolicy.sample() → returns deterministic mean only."""
#     def __init__(self, policy):
#         super().__init__()
#         self.policy = policy

#     def forward(self, state: torch.Tensor) -> torch.Tensor:
#         _, _, mean = self.policy.sample(state)
#         return mean          # [B, action_dim]


# class AdversaryExport(nn.Module):
#     """Same wrapper for the adversary."""
#     def __init__(self, policy):
#         super().__init__()
#         self.policy = policy

#     def forward(self, state: torch.Tensor) -> torch.Tensor:
#         _, _, mean = self.policy.sample(state)
#         return mean

class DeterministicPolicyExport(nn.Module):
    """
    Exports only the deterministic mean action, bypassing Normal sampling.
    Equivalent to the third return value of GaussianPolicy.sample() but
    traceable cleanly by torch.onnx.
    """
    def __init__(self, policy):
        super().__init__()
        self.mu_head      = policy.mu_head
        self.log_std_head = policy.log_std_head   # not used, but keep ref
        self.action_scale = policy.action_scale
        self.action_bias  = policy.action_bias

    def forward(self, state: torch.Tensor) -> torch.Tensor:
        mean = self.mu_head(state)                          # raw mean
        mean = torch.tanh(mean)                             # squash
        mean = mean * self.action_scale + self.action_bias  # rescale
        return mean


class CriticExport(nn.Module):
    """Wraps QNetwork(state, action, disturbance) → (q1, q2)."""
    def __init__(self, qnet):
        super().__init__()
        self.qnet = qnet

    def forward(self,
                state:       torch.Tensor,
                action:      torch.Tensor,
                disturbance: torch.Tensor):
        q1, q2 = self.qnet(state, action, disturbance)
        return q1, q2


# --------------------------------------------------------------------------- #
# Main                                                                         #
# --------------------------------------------------------------------------- #

def build_agent(args):
    if args.config_yaml:
        with open(args.config_yaml) as f:
            cfg_dict = yaml.safe_load(f)
        ns = SimpleNamespace(**{k: v for sec in cfg_dict.values() for k, v in sec.items()})
    else:
        # Sensible defaults matching the Python node
        ns = SimpleNamespace(
            annealing=False, updatePeriodEps=1000, maxUpdates=500000,
            numObstacles=args.num_obstacles, numEnvs=1,
            randomSeed=0, maxSteps=200, batchSize=256,
            memoryCapacity=10000, criticArchitecture=[512,512],
            actorArchitecture=[256, 256], actType="ReLU",
            gamma=0.95, updatePeriodGamma=200, gammaDecay=0.5,
            eps=0.95, epsEnd=0.05, epsDecay=0.5, epsEnd2=0.05,
            learningRate=1e-3, learningRateActor=1e-4,
            updatePeriodLr=1, learningRateDecay=0.5,
            maxModel=5, numCritics=args.num_critics,
            selectWorstQ=True, findMaxQ=False, simMaxQ=False,
            timeStep=0.05, tau=0.01, hardUpdate=1, softUpdate=True,
            render=False, double=True, reward=-1, penalty=1,
            alpha=0.2, policy="Gaussian", targetUpdateInterval=1,
            autoAlphaTuning=True,
        )

    state_dim  = 6 + 3 * args.num_obstacles
    action_dim = 3

    GAMMA_END        = 0.999999 if getattr(ns, 'annealing', False) else 0.999
    EPS_PERIOD       = int(ns.updatePeriodEps / 10) if getattr(ns, 'annealing', False) else ns.updatePeriodEps
    EPS_RESET_PERIOD = ns.updatePeriodEps if getattr(ns, 'annealing', False) else ns.maxUpdates

    config_params = dict(
        DEVICE="cpu", ENV_NAME="cont-obs-avoid-v0",
        NUM_ENVS=1, SEED=ns.randomSeed,
        MAX_UPDATES=ns.maxUpdates, MAX_EP_STEPS=ns.maxSteps,
        BATCH_SIZE=ns.batchSize, MEMORY_CAPACITY=ns.memoryCapacity,
        C_ARCHITECTURE=ns.criticArchitecture, A_ARCHITECTURE=ns.actorArchitecture,
        ACTIVATION=ns.actType, GAMMA=ns.gamma,
        GAMMA_PERIOD=ns.updatePeriodGamma, GAMMA_END=GAMMA_END, GAMMA_DECAY=ns.gammaDecay,
        EPSILON=ns.eps, EPS_END=ns.epsEnd, EPS_PERIOD=EPS_PERIOD,
        EPS_DECAY=ns.epsDecay, EPS_RESET_PERIOD=EPS_RESET_PERIOD,
        LR_C=ns.learningRate, LR_C_END=ns.learningRate*0.5,
        LR_C_PERIOD=ns.updatePeriodLr, LR_C_DECAY=ns.learningRateDecay,
        LR_A=ns.learningRateActor, LR_A_END=ns.learningRateActor*0.5,
        LR_A_PERIOD=ns.updatePeriodLr, LR_A_DECAY=ns.learningRateDecay,
        MAX_MODEL=ns.maxModel, NUM_CRITICS=ns.numCritics,
        SELECT_WORST_Q=ns.selectWorstQ, FIND_MAX_Q=ns.findMaxQ, SIM_MAX_Q=ns.simMaxQ,
        TIME_STEP=ns.timeStep, TAU=ns.tau, HARD_UPDATE=ns.hardUpdate,
        SOFT_UPDATE=ns.softUpdate, RENDER=ns.render, DOUBLE=ns.double,
        REWARD=ns.reward, PENALTY=ns.penalty, ALPHA=ns.alpha,
        POLICY=ns.policy, TARGET_UPDATE_INTERVAL=ns.targetUpdateInterval,
        AUTO_ALPHA_TUNING=ns.autoAlphaTuning,
    )
    CONFIG = ceConfig(**config_params)

    action_space      = gymnasium.spaces.Box(-np.ones(action_dim, dtype=np.float32),
                                              np.ones(action_dim, dtype=np.float32))
    disturbance_space = deepcopy(action_space)

    c_dimList = [state_dim] + CONFIG.C_ARCHITECTURE + [action_dim]
    a_dimList = [state_dim] + CONFIG.A_ARCHITECTURE + [action_dim]

    agent = SAC(CONFIG, c_dimList=c_dimList, a_dimList=a_dimList,
                action_space=action_space, disturbance_space=disturbance_space)
    agent.load_best_models(args.best_dir, evaluate=True)

    return agent, state_dim, action_dim


def export(args):
    os.makedirs(args.out_dir, exist_ok=True)
    agent, state_dim, action_dim = build_agent(args)

    dummy_state   = torch.zeros(1, state_dim)
    dummy_action  = torch.zeros(1, action_dim)
    dummy_disturb = torch.zeros(1, action_dim)

    # ── Protagonist ──────────────────────────────────────────────────────────
    pro_wrap = DeterministicPolicyExport(agent.protagonist).eval()
    out_path = os.path.join(args.out_dir, "protagonist.onnx")
    torch.onnx.export(
        pro_wrap, dummy_state, out_path,
        input_names=["state"], output_names=["action"],
        dynamic_axes={"state": {0: "batch"}, "action": {0: "batch"}},
        opset_version=17,
    )

    # ── Adversary ────────────────────────────────────────────────────────────
    adv_wrap = DeterministicPolicyExport(agent.adversary).eval()
    out_path = os.path.join(args.out_dir, "adversary.onnx")
    torch.onnx.export(
        adv_wrap, dummy_state, out_path,
        input_names=["state"], output_names=["disturbance"],
        dynamic_axes={"state": {0: "batch"}, "disturbance": {0: "batch"}},
        opset_version=17,
    )

    # ── Critics ──────────────────────────────────────────────────────────────
    for i, critic in enumerate(agent.critics):
        critic_wrap = CriticExport(critic).eval()
        out_path = os.path.join(args.out_dir, f"critic_{i}.onnx")
        torch.onnx.export(
            critic_wrap,
            (dummy_state, dummy_action, dummy_disturb),
            out_path,
            input_names=["state", "action", "disturbance"],
            output_names=["q1", "q2"],
            dynamic_axes={k: {0: "batch"} for k in ["state","action","disturbance","q1","q2"]},
            opset_version=17,
        )
        print(f"Exported → {out_path}")

    print("Done.")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--best_dir",      required=True, help="Dir with protagonist.pt, adversary.pt, critic_N.pt")
    p.add_argument("--out_dir",       required=True, help="Where to write .onnx files")
    p.add_argument("--num_obstacles", type=int, default=3)
    p.add_argument("--num_critics",   type=int, default=3)
    p.add_argument("--config_yaml",   default=None,  help="Optional path to config.yaml")
    export(p.parse_args())
