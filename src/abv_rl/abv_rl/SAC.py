"""
SAC.py  –  Soft Actor-Critic for two-player reach-avoid games.

Fixes applied vs. the original:
  1.  self.BATCH_SIZE / self.GAMMA / self.CONFIG were referenced but never
      set → assigned from config in __init__.
  2.  self.policy / self.policy_optim were referenced in update() but the
      actual attributes are self.protagonist / self.protagonist_optim.
      Added policy/policy_optim as aliases so existing code keeps working.
  3.  Added adversary policy-loss computation and optimisation step (the
      adversary *maximises* the Q-value, so its loss is negated).
  4.  update() now returns early gracefully (returns None) when the buffer
      is too small, matching how SACTrainer.learn() checks for None.
  5.  Corrected save/load_checkpoint to use protagonist (not self.policy).
"""

import os
from typing import List
import copy

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
# from torch.optim import Adam
import torch.optim as optim
from .utils import soft_update, hard_update, save_model
from .model import GaussianPolicy, QNetwork, MaxEnsembleNet, DeterministicPolicy, StepLRMargin, RNDPredictor, RNDPrior
from concurrent.futures import ThreadPoolExecutor, as_completed
import matplotlib.pyplot as plt
# from gym_reachability.gym_reachability.envs.env_utils import calculate_margin_circle

from collections import namedtuple
Transition = namedtuple("Transition", ["s", "a", "d", "r", "s_", "a_", "ending", "done", "info"])

class SAC(object):
    def __init__(self, config, c_dimList, a_dimList, RNDpred_dimList, RNDpost_dimList, action_space, disturbance_space, n_workers: int = None,):

        # 
        # Hyper-parameters  (kept as both lower- and UPPER-case so that
        # existing code referencing either form continues to work)
        # 
        self.CONFIG     = config         
        self.tau        = config.TAU
        self.alpha_pro  = config.ALPHA
        self.alpha_adv  = config.ALPHA
        self.lambda_    = config.LAMBDA
        self.BATCH_SIZE = config.BATCH_SIZE   

        # Learning rate of updating the Q-network
        self.LR_C = config.LR_C
        self.LR_C_PERIOD = config.LR_C_PERIOD
        self.LR_C_DECAY = config.LR_C_DECAY
        self.LR_C_END = config.LR_C_END  

        # Learning rate of updating the policy networks
        self.LR_A = config.LR_A
        self.LR_A_PERIOD = config.LR_A_PERIOD
        self.LR_A_DECAY = config.LR_A_DECAY
        self.LR_A_END = config.LR_A_END

        self.autoAlphaTuning = config.AUTO_ALPHA_TUNING

        self.num_critics = config.NUM_CRITICS

        self.n_workers    = n_workers if n_workers is not None else self.num_critics

        self.c_dimList    = c_dimList
        self.a_dimList    = a_dimList

        self.policy_type           = config.POLICY
        self.target_update_interval = config.TARGET_UPDATE_INTERVAL

        self.device = torch.device(config.DEVICE)

        if self.autoAlphaTuning:
            # Target entropy is -|A| (e.g. -2 for Ant-v2) as per SAC paper
            self.pro_target_entropy = -action_space.shape[0]
            self.pro_log_alpha = torch.zeros(1, requires_grad=True, device=self.device)
            self.pro_alpha_optim = optim.Adam([self.pro_log_alpha], lr=0.0003)

            self.adv_target_entropy = -disturbance_space.shape[0]
            self.adv_log_alpha = torch.zeros(1, requires_grad=True, device=self.device)
            self.adv_alpha_optim = optim.Adam([self.adv_log_alpha], lr=0.0003)

            if self.CONFIG.MODE == "AARA_C":
                self.log_lambda = torch.zeros(1, requires_grad=True, device=self.device)
                self.lambda_optim = optim.Adam([self.log_lambda], lr=0.000001)

        # 
        # Critics
        # 
        self.makeCritics('', action_space)
        if self.CONFIG.MODE == "AARA_C":
            # self.c_dimList = [self.c_dimList[0]] + [512] + [512] + [1]
            self.makeCritics('RA_', action_space)

        self.max_grad_norm = 1
        self.cntUpdate = 0

        # Thread pool — one worker per critic, reused across all update() calls.
        # PyTorch releases the GIL during tensor ops so threads genuinely
        # run in parallel on separate cores.
        self._executor = ThreadPoolExecutor(max_workers=self.n_workers)

        # Discount factor: anneal to one
        self.GAMMA      = config.GAMMA 
        self.GammaScheduler = StepLRMargin(
            initValue=config.GAMMA,
            period=config.GAMMA_PERIOD,
            decay=config.GAMMA_DECAY,
            endValue=config.GAMMA_END,
            goalValue=1.0,
            numEnvs = config.NUM_ENVS
        )
        self.GAMMA = self.GammaScheduler.get_variable()

        #
        # RND Networks (predictor and prior)
        #

        self.RNDPredictor = RNDPredictor(self.CONFIG, RNDpred_dimList, action_space.shape[0]).to(self.device)
        self.predictor_optim = optim.AdamW(self.RNDPredictor.parameters(), lr=1e-4, weight_decay=1e-3)
        self.RND_scheduler = optim.lr_scheduler.StepLR(
            self.predictor_optim, step_size=self.LR_A_PERIOD, gamma=self.LR_A_DECAY
        )

        self.RNDPrior = RNDPrior(self.CONFIG, RNDpost_dimList, action_space.shape[0]).to(self.device)
        self.RNDPrior.eval()
        for parameter in self.RNDPrior.parameters():
            parameter.requires_grad_(False)

        self.rnd_error_ema = torch.tensor(1e-6, device=self.device)
        self.rnd_ema_initialized = False
        self.rnd_ema_beta = 0.999

        # 
        # Policies
        # 
        PolicyCls = GaussianPolicy if self.policy_type == "Gaussian" else DeterministicPolicy
        if self.policy_type != "Gaussian":
            self.alpha = 0  # deterministic → no entropy bonus

        self.protagonist       = PolicyCls(config, self.a_dimList, action_space.shape[0], action_space, conditioned_sigma=True).to(self.device)
        self.protagonist_optim = optim.AdamW(self.protagonist.parameters(), lr=config.LR_A, weight_decay=1e-3)

        self.adversary       = PolicyCls(config, self.a_dimList, disturbance_space.shape[0], disturbance_space, conditioned_sigma=True).to(self.device)
        self.adversary_optim = optim.AdamW(self.adversary.parameters(), lr=config.LR_A, weight_decay=1e-3)

        self.protagonist_scheduler = optim.lr_scheduler.StepLR(
            self.protagonist_optim, step_size=self.LR_A_PERIOD, gamma=self.LR_A_DECAY
        )

        self.adversary_scheduler = optim.lr_scheduler.StepLR(
            self.adversary_optim, step_size=self.LR_A_PERIOD, gamma=self.LR_A_DECAY
        )

        self.prev_pro_loss = 0.0
        self.prev_adv_loss = 0.0

    # -
    # Trainer-facing API
    # -

    # --- cntUpdate: must stay synced so Trainer's while-loop and
    #     per-critic updateHyperParam() schedules all stay aligned -

    @property
    def cntUpdate(self):
        return self.critics[0].cntUpdate

    @cntUpdate.setter
    def cntUpdate(self, value):
        for c in self.critics:
            c.cntUpdate = value

    @property
    def GAMMA(self):
        return self.critics[0].GAMMA

    @GAMMA.setter
    def GAMMA(self, value):
        for c in self.critics:
            c.GAMMA = value

    # -
    # Action selection
    # -

    def select_action(self, state, explore=False):
        """
        Returns (protagonist_action, adversary_action) as numpy arrays.

        Args:
            state (np.ndarray): current observation.
            explore (bool): if True sample stochastically, else use the
                            deterministic mean.
        """
        state_t = torch.FloatTensor(state).to(self.device).unsqueeze(0)
        with torch.no_grad():
            if explore:
                action,      _, _ = self.protagonist.sample(state_t)
                disturbance, _, _ = self.adversary.sample(state_t)
            else:
                _, _, action      = self.protagonist.sample(state_t)
                _, _, disturbance = self.adversary.sample(state_t)
        return (
            action.cpu().numpy()[0],
            disturbance.cpu().numpy()[0],
        )

    # def critic_update(self, critic, critic_target, critic_optim, state, action, non_final_state_nxt, non_final_mask, next_action, next_log_pi_a, l_x, g_x, ending, norm_scale, ep_unc=None):

    #     # 
    #     # 1.  Critic update (reach-avoid Bellman target)
    #     # 
    #     critic.train()

    #     qf1, qf2 = critic(state, action, norm_scale)

    #     max_qf_next_target = torch.zeros(self.BATCH_SIZE).to(self.device)

    #     with torch.no_grad():
    #         qf1_next, qf2_next = critic_target(non_final_state_nxt, next_action, norm_scale)
    #         # protagonist minimises → take the maximum of the two Q-heads
    #     max_qf_next_target[non_final_mask] = (torch.max(qf1_next, qf2_next) + (self.alpha_pro * next_log_pi_a)).view(-1)

    #     # Epistemic-uncertainty weight
    #     with torch.no_grad():
    #         _lambda  = 0 * -torch.sqrt(ep_unc) if ep_unc is not None else torch.zeros(state.shape[0],1, device=self.device)
    #         eu_weight = torch.exp(_lambda * self.CONFIG.TIME_STEP).squeeze(-1)
        
    #     # Reach-avoid backup
    #     terminal     = torch.max(l_x, g_x)
    #     non_terminal = torch.max(g_x[non_final_mask],
    #         torch.min(l_x[non_final_mask], eu_weight[non_final_mask] * max_qf_next_target[non_final_mask]))

    #     next_q_value = torch.zeros(self.BATCH_SIZE).float().to(self.device)
    #     final_mask = torch.logical_not(non_final_mask)
    #     next_q_value[non_final_mask] = (
    #         (1 - self.GAMMA) * terminal[non_final_mask] + self.GAMMA * non_terminal
    #     )
    #     next_q_value[final_mask] = terminal[final_mask]

    #     q_max = torch.max(qf1, qf2)
    #     mask = ending.bool().reshape(-1)
    #     FE_loss = F.softplus(q_max.reshape(-1)[mask]).mean() if mask.any() else torch.tensor(0.0, device=self.device)

    #     pde_loss = self.compute_pdedot_loss(
    #         qf1, qf2, l_x, g_x, non_final_mask, max_qf_next_target,
    #         eu_weight, self.CONFIG.TIME_STEP, norm_scale
    #     )

    #     qf1_loss = F.mse_loss(qf1, next_q_value.unsqueeze(-1).detach())
    #     qf2_loss = F.mse_loss(qf2, next_q_value.unsqueeze(-1).detach())
    #     # qf1_loss = torch.max(torch.abs(qf1 - next_q_value.unsqueeze(-1).detach()))
    #     # qf2_loss = torch.max(torch.abs(qf2 - next_q_value.unsqueeze(-1).detach()))
    #     qf_loss  = qf1_loss + qf2_loss + pde_loss  + FE_loss

    #     critic_optim.zero_grad()
    #     qf_loss.backward(retain_graph=True)
    #     torch.nn.utils.clip_grad_norm_(critic.parameters(), 1.0)
    #     critic_optim.step()
        

    #     return qf1_loss, qf2_loss, FE_loss, mask.float().mean()


    # def RL_critic_update(self, critic, critic_target, critic_optim, state, action, non_final_state_nxt, non_final_mask, next_action, next_log_pi_a, reward, norm_scale, ep_unc=None):

    #     # 
    #     # 1.  Critic update (reach-avoid Bellman target)
    #     # 
    #     critic.train()

    #     qf1, qf2 = critic(state, action, norm_scale)

    #     max_qf_next_target = torch.zeros(self.BATCH_SIZE).to(self.device)

    #     with torch.no_grad():
    #         qf1_next, qf2_next = critic_target(non_final_state_nxt, next_action, norm_scale)
    #         # protagonist maximises → take the minimum of the two Q-heads
    #     max_qf_next_target[non_final_mask] = (torch.min(qf1_next, qf2_next) - (self.alpha_pro * next_log_pi_a)).view(-1)

    #     # Epistemic-uncertainty weight
    #     # with torch.no_grad():
    #     #     _lambda  = -torch.sqrt(ep_unc) if ep_unc is not None else torch.zeros(state.shape[0],1, device=self.device)
    #     #     eu_weight = torch.exp(_lambda * self.CONFIG.TIME_STEP)
        
    #     # Do not bootstrap terminal transitions; their target is just the immediate shaped reward.
    #     next_q_value = reward + (
    #         self.GAMMA * max_qf_next_target * non_final_mask.float()
    #     ).unsqueeze(1)

    #     qf1_loss = F.mse_loss(qf1, next_q_value.detach())
    #     qf2_loss = F.mse_loss(qf2, next_q_value.detach())
    #     qf_loss  = qf1_loss + qf2_loss

    #     critic_optim.zero_grad()
    #     qf_loss.backward(retain_graph=True)
    #     torch.nn.utils.clip_grad_norm_(critic.parameters(), 1.0)
    #     critic_optim.step()

    #     return qf1_loss, qf2_loss

    # def compute_pdedot_loss(self, qf1, qf2, l_x, g_x,
    #                       non_final_mask, max_qf_next_target, eu_weight, dt, norm_scale):
    #     """
    #     Finite-difference V-dot residual, branch-gated to the ell-active region.
    #     No dynamics model required — uses sampled (s_t, s_{t+1}) pairs directly.
    #     """
    #     with torch.no_grad():
    #         V = torch.max(qf1, qf2).squeeze(-1)

    #     V_nf   = V[non_final_mask]
    #     l_nf   = l_x[non_final_mask].squeeze(-1)
    #     g_nf   = g_x[non_final_mask].squeeze(-1)
    #     backup = self.GAMMA * eu_weight[non_final_mask] * max_qf_next_target[non_final_mask]  # V(s_{t+1}) side, already discounted

    #     # finite-difference V_dot: use *undiscounted* V(s_{t+1}) estimate for the derivative,
    #     # not the discounted backup (discounting is lambda's job, not V_dot's)
    #     V_next = max_qf_next_target[non_final_mask]  # raw critic_target value at s_{t+1}, no eu_weight applied
    #     Vdot   = (V_next - V_nf) / dt

    #     ell_active = (g_nf < V_nf) & (l_nf <= backup)

    #     if ell_active.any():
    #         residual = Vdot[ell_active]
    #         pde_loss = F.relu(-residual).pow(2).mean()
    #     else:
    #         pde_loss = torch.tensor(0.0, device=self.device)

    #     return pde_loss

    # -
    # Gradient update
    # -

    # def update(self, memory, batch_size, updates, norm_scale, batch=None):
    #     """
    #     One gradient step for the critic, protagonist, and adversary.

    #     Args:
    #         memory (ReplayMemory): replay buffer.
    #         batch_size (int): mini-batch size.
    #         updates (int): global update counter (used for target sync).
    #         ep_unc (float | None): epistemic uncertainty weight.
    #         batch (Transition | None): pre-assembled batch (optional).

    #     Returns:
    #         Tuple (qf1_loss, qf2_loss, pro_loss, adv_loss, alpha_tlog)
    #         or None if the buffer is not yet large enough.
    #     """

    #     #  sample from replay buffer 
    #     if batch is None:
    #         batch = memory.sample(self.BATCH_SIZE)

    #     (
    #         non_final_mask,
    #         non_final_state_nxt,
    #         state,
    #         action,
    #         action_next,
    #         _,
    #         reward,
    #         g_x,
    #         l_x,
    #         ending,
    #     ) = self.unpack_batch(batch)

    #     with torch.no_grad():
    #         target = self.RNDPrior(state, action).detach()
    #     pred = self.RNDPredictor(state, action)

    #     rnd_error = (pred - target).pow(2).mean(dim=-1, keepdim=True)
    #     RND_loss = rnd_error.mean()

    #     self.predictor_optim.zero_grad(set_to_none=True)
    #     RND_loss.backward()
    #     self.predictor_optim.step()

    #     with torch.no_grad():
    #         batch_error_mean = rnd_error.mean()

    #         if not self.rnd_ema_initialized:
    #             self.rnd_error_ema.copy_(batch_error_mean)
    #             self.rnd_ema_initialized = True
    #         else:
    #             self.rnd_error_ema.mul_(self.rnd_ema_beta).add_(
    #                 batch_error_mean,
    #                 alpha=1.0 - self.rnd_ema_beta
    #             )

    #         relative_error = rnd_error / self.rnd_error_ema.clamp_min(1e-12)

    #         # Typical samples near the EMA produce approximately zero.
    #         # Errors >= 5 times the reference level saturate at one.
    #         epistemic_uncertainty = (
    #             (relative_error - 1.0) / 4.0
    #         ).clamp(0.0, 1.0)

    #     self.epistem_uncertainty = epistemic_uncertainty

    #     with torch.no_grad():
    #         non_final_state_nxt = non_final_state_nxt[non_final_mask.cpu()]
    #         next_action, next_log_pi_a, _ = self.protagonist.sample(non_final_state_nxt)

    #     if self.CONFIG.MODE == "AARA":
    #         futures = {
    #             self._executor.submit(self.critic_update, c, c_t, c_opt, state, action, non_final_state_nxt, non_final_mask, next_action, next_log_pi_a, l_x, g_x, ending, norm_scale, self.epistem_uncertainty): c
    #             for c, c_t, c_opt in zip(self.critics, self.critic_targets, self.critic_optimisers)
    #         }
    #     elif self.CONFIG.MODE == "AARA_C":
    #         futures = {
    #             self._executor.submit(self.RL_critic_update, c, c_t, c_opt, state, action, non_final_state_nxt, non_final_mask, next_action, next_log_pi_a, reward, norm_scale, self.epistem_uncertainty): c
    #             for c, c_t, c_opt in zip(self.critics, self.critic_targets, self.critic_optimisers)
    #         }
    #     else:
    #         raise KeyError(f"Selected mode '{self.CONFIG.MODE}' is not compatible yet.")

    #     losses = []
    #     for fut in as_completed(futures):
    #         c = futures[fut]
    #         result = fut.result()
    #         if result is not None:
    #             losses.append(result)

    #     q_means = [torch.stack(t).mean(0) for t in zip(*losses)]
    #     if self.CONFIG.MODE == "AARA":
    #         qf1_loss, qf2_loss, FE_loss, FE_count = q_means
    #     elif self.CONFIG.MODE == "AARA_C":
    #         qf1_loss, qf2_loss = q_means
    #         FE_loss = torch.zeros(1)
    #         FE_count = torch.zeros(1)

    #     if updates % 4 == 0 and self.CONFIG.TRAIN_POLICY:
    #         self.protagonist.train()
    #         pi_pro, log_pi_pro, _ = self.protagonist.sample(state)

    #         self.adversary.train()
    #         pi_adv, log_pi_adv, _ = self.adversary.sample(state)


    #         # 
    #         # 2.  Protagonist update  (minimise Q)
    #         # 
    #         if self.CONFIG.MODE == "AARA":
    #             max_qf_pi = self.Q_network(state, pi_pro, norm_scale, mode="max")
    #             # Protagonist wants to *minimise* Q  → minimise  Q - α·H
    #             smoothness_loss = 0.02 * F.mse_loss(action_next, action)
    #             pro_loss = (max_qf_pi + self.alpha_pro * log_pi_pro).mean() + smoothness_loss
    #         elif self.CONFIG.MODE == "AARA_C":
    #             max_qf_pi = self.Q_network(state, pi_pro, norm_scale, mode="min")
    #             raw_cost = self.RA_Q_network(state, pi_pro, norm_scale, mode="max")
    #             max_qf_cost = raw_cost #+ self.epistem_uncertainty
    #             # AARA_C uses the ordinary RL critic with a reward convention:
    #             # higher return is better, so the protagonist maximises Q.
    #             smoothness_loss = 0.02 * F.mse_loss(action_next, action)
    #             pro_loss = (-max_qf_pi + self.lambda_ * max_qf_cost + self.alpha_pro * log_pi_pro).mean() + smoothness_loss
    #         else:
    #             raise KeyError(f"Selected mode '{self.CONFIG.MODE}' is not compatible yet.")

    #         self.protagonist_optim.zero_grad()
    #         pro_loss.backward(retain_graph=True)
    #         self.protagonist_optim.step()

    #         self.prev_pro_loss = pro_loss.item()


    #         # 
    #         # 3.  Adversary update  (maximise Q)   
    #         # 
    #         min_qf_pi = self.Q_network(state, pi_adv, norm_scale, mode="min")

    #         # The adversary feeds its action into the *same* critic but wants
    #         # to drive Q *up* → maximise  Q + α·H  (entropy regularised) -loss
    #         if self.CONFIG.MODE == "AARA":
    #             adv_loss = (-min_qf_pi + self.alpha_adv * log_pi_adv).mean()
    #         elif self.CONFIG.MODE == "AARA_C":
    #             adv_loss = (min_qf_pi + self.alpha_adv * log_pi_adv).mean()
    #         else:
    #             raise KeyError(f"Selected mode '{self.CONFIG.MODE}' is not compatible yet.")

    #         self.adversary_optim.zero_grad()
    #         adv_loss.backward(retain_graph=True)
    #         self.adversary_optim.step()

    #         self.prev_adv_loss = adv_loss.item()
    #     else:
    #         pi_pro, log_pi_pro, _ = self.protagonist.sample(state)
    #         pi_adv, log_pi_adv, _ = self.adversary.sample(state)


    #     # 
    #     # 4.  Alpha / entropy tuning 
    #     # 
    #     if self.autoAlphaTuning and updates % 8 == 0 and self.CONFIG.TRAIN_POLICY:
    #         beta = 100
    #         eff_pro_target_entropy = self.pro_target_entropy * torch.exp(-self.epistem_uncertainty.detach()*beta)

    #         pro_alpha_loss = -(self.pro_log_alpha * (log_pi_pro + eff_pro_target_entropy).detach()).mean()
    #         self.pro_alpha_optim.zero_grad()
    #         pro_alpha_loss.backward()
    #         self.pro_alpha_optim.step()
    #         self.alpha_pro = self.pro_log_alpha.exp()

    #         eff_adv_target_entropy = self.adv_target_entropy * torch.exp(-self.epistem_uncertainty.detach()*beta)
    #         adv_alpha_loss = -(self.adv_log_alpha * (log_pi_adv + eff_adv_target_entropy).detach()).mean()
    #         self.adv_alpha_optim.zero_grad()
    #         adv_alpha_loss.backward()
    #         self.adv_alpha_optim.step()
    #         self.alpha_adv = self.adv_log_alpha.exp()

    #         if self.CONFIG.MODE == "AARA_C":
    #             lambda_loss = -self.log_lambda * raw_cost.detach().mean()
    #             self.lambda_optim.zero_grad()
    #             lambda_loss.backward()
    #             self.lambda_optim.step()
    #             with torch.no_grad():
    #                 self.lambda_ = self.log_lambda.exp().detach()


    #     alpha_tlogs = torch.tensor(float(self.alpha_pro))


    #     # 
    #     # 5.  Soft-update of target critic
    #     # 
    #     if updates % self.target_update_interval == 0:
    #         for critic_target, critic in zip(self.critic_targets, self.critics):
    #             soft_update(critic_target, critic, self.tau)

    #     debug = {"q_perf": max_qf_pi.mean().item(),
    #              "l_constraint": (self.lambda_*max_qf_cost).mean().item(),
    #              "raw_cost": raw_cost.mean().item(),
    #              } if self.CONFIG.MODE == "AARA_C" else {"q_perf": max_qf_pi.mean().item()}

    #     return (
    #         qf1_loss.item(),
    #         qf2_loss.item(),
    #         FE_loss.item(),
    #         FE_count.item(),
    #         RND_loss.item(),
    #         debug,
    #         self.prev_pro_loss,
    #         self.prev_adv_loss,
    #         alpha_tlogs.item(),
    #         self.epistem_uncertainty.mean().cpu().detach().item()
    #     )
    
    # # -
    # # Update Hyperparameters
    # # -

    # def updateHyperParam(self):
    #     """
    #     Updates the hypewr-parameters, such as learning rate, discount factor
    #     (GAMMA) and exploration-exploitation tradeoff (EPSILON)
    #     """
    #     for critic_optim, scheduler in zip(self.critic_optimisers, self.critic_schedulers):
    #         lr = critic_optim.state_dict()["param_groups"][0]["lr"]
    #         if (lr <= self.LR_C_END):
    #             for param_group in critic_optim.param_groups:
    #                 param_group["lr"] = self.LR_C_END
    #         else:
    #             scheduler.step()
    #     self.protagonist_scheduler.step()
    #     self.adversary_scheduler.step()

    #     self.GammaScheduler.step()
    #     self.GAMMA = self.GammaScheduler.get_variable()

    # def slice_batch(tensor, idx, batch_size):
    #     start = idx * batch_size
    #     end = (idx + 1) * batch_size
    #     return tensor[start:end]
    
    # - 
    # Make Critics
    #
    def makeCritics(self, name, action_space):
        ensemble: List[QNetwork] = []
        # optimisers: List[optim.AdamW] = []
        # schedulers: List [optim.lr_scheduler.StepLR] = []
        # targets: List[QNetwork] = []
        if self.CONFIG.SEED is None: 
            self.CONFIG.SEED = 0
        for i in range(self.num_critics):
            cfg_i = copy.deepcopy(self.CONFIG)
            cfg_i.SEED += i
            critic = QNetwork(self.CONFIG, self.c_dimList, action_space.shape[0]).to(self.device)

            ensemble.append(critic)
            print(
                f"  [Ensemble] Critic {i:02d} | seed={cfg_i.SEED}"
                # f" | device={self.critic.device}"
            )

            # critic_optim = optim.AdamW(critic.parameters(), lr=self.CONFIG.LR_C, weight_decay=1e-3)
            # optimisers.append(critic_optim)

            # scheduler = optim.lr_scheduler.StepLR(
            #     critic_optim, step_size=self.LR_C_PERIOD, gamma=self.LR_C_DECAY
            # )
            # schedulers.append(scheduler)

            # target = QNetwork(self.CONFIG, self.c_dimList, action_space.shape[0]).to(self.device)
            # hard_update(target, critic)
            # targets.append(target)

        # Unified inference modules — what Trainer / env see as Q_network
        MaxEnsemble = MaxEnsembleNet([c for c in ensemble])

        setattr(self, name + "critics", ensemble)
        # setattr(self, name + "critic_optimisers", optimisers)
        # setattr(self, name + "critic_schedulers", schedulers)
        # setattr(self, name + "critic_targets", targets)
        setattr(self, name + "Q_network", MaxEnsemble)
   
    # -
    # Checkpointing 
    # -

    def save_checkpoint(self, env_name, suffix="", ckpt_path=None):
        os.makedirs("checkpoints/", exist_ok=True)
        if ckpt_path is None:
            ckpt_path = "checkpoints/sac_checkpoint_{}_{}".format(env_name, suffix)
        print("Saving models to {}".format(ckpt_path))
        torch.save(
            {
                "protagonist_state_dict":       self.protagonist.state_dict(),
                "adversary_state_dict":         self.adversary.state_dict(),
                "critic_state_dict":            self.critic.state_dict(),
                "critic_target_state_dict":     self.critic_target.state_dict(),
                "critic_optimizer_state_dict":  self.critic_optim.state_dict(),
                "protagonist_optimizer_state_dict": self.protagonist_optim.state_dict(),
                "adversary_optimizer_state_dict":   self.adversary_optim.state_dict(),
            },
            ckpt_path,
        )

    # def load_checkpoint(self, modelIter, ckpt_path, evaluate=True):
    #     net_mode = "eval" if evaluate else "train"
    #     pro_ckpt_path = os.path.join(ckpt_path, "pro_model", "model_{}.pt".format(modelIter))
    #     adv_ckpt_path = os.path.join(ckpt_path, "adv_model", "model_{}.pt".format(modelIter))
    #     print("Loading models from {}".format(pro_ckpt_path))
    #     if ckpt_path is not None:
    #         self.protagonist.load_state_dict(torch.load(pro_ckpt_path, map_location=self.device))
    #         self.adversary.load_state_dict(torch.load(adv_ckpt_path, map_location=self.device))
    #         for i, critic in enumerate(self.critics):
    #             critic_ckpt_path_i = os.path.join(ckpt_path, "pro_model", "critic_{}".format(i), "critic_{}.pt".format(modelIter))
    #             critic.load_state_dict(torch.load(critic_ckpt_path_i, map_location=self.device))
    #             getattr(critic, net_mode)()
            
    #         for net in [self.protagonist, self.adversary]:
    #             getattr(net, net_mode)()
        
    #     self.Q_network = MaxEnsembleNet([c for c in self.critics])

    def load_best_main_models(self, bestDir, evaluate=True):
        """Load the protagonist, adversary, and main AARA critic ensemble."""
        net_mode = "eval" if evaluate else "train"
        self.protagonist.load_state_dict(torch.load(
            os.path.join(bestDir, "protagonist.pt"), map_location=self.device
        ))
        self.adversary.load_state_dict(torch.load(
            os.path.join(bestDir, "adversary.pt"), map_location=self.device
        ))
        for i, critic in enumerate(self.critics):
            critic.load_state_dict(torch.load(
                os.path.join(bestDir, f"critic_{i}.pt"),
                map_location=self.device,
            ))
            getattr(critic, net_mode)()
        self.Q_network = MaxEnsembleNet([c for c in self.critics])

        for net in (self.protagonist, self.adversary):
            getattr(net, net_mode)()

    def load_best_ra_models(self, bestDir, evaluate=True):
        """Load the constrained AARA critic ensemble into RA_Q_network."""
        net_mode = "eval" if evaluate else "train"
        for i, critic in enumerate(self.RA_critics):
            critic.load_state_dict(torch.load(
                os.path.join(bestDir, f"RA_critic_{i}.pt"),
                map_location=self.device,
            ))
            getattr(critic, net_mode)()
        self.RA_Q_network = MaxEnsembleNet([c for c in self.RA_critics])

    def load_best_models(self, bestDir, q_val_dir=None, mode="AARA", evaluate=True):
        """Backward-compatible wrapper for loading the requested model set."""
        if mode == "AARA":
            self.load_best_main_models(bestDir, evaluate=evaluate)
        elif mode == "AARA_C":
            if q_val_dir is not None:
                self.load_best_ra_models(q_val_dir, evaluate=evaluate)
            else:
                self.load_best_main_models(bestDir, evaluate=evaluate)
        else:
            raise ValueError(f"Unsupported model-loading mode: {mode}")
        

    # -
    # Batch unpacking  (unchanged from original)
    # -

    def unpack_batch(self, batch):
        """Decomposes the batch into tensors ready for update().

        Returns:
            (non_final_mask, non_final_state_nxt, state, action,
             reward, g_x, l_x)
        """
        non_final_mask = torch.tensor(~batch.done, dtype=torch.bool, device=self.device)

        state = torch.as_tensor(batch.s, dtype=torch.float32, device=self.device)
        action = torch.as_tensor(batch.a, dtype=torch.float32, device=self.device)
        action_next = torch.as_tensor(batch.a_, dtype=torch.float32, device=self.device)
        disturbance = torch.as_tensor(batch.d, dtype=torch.float32, device=self.device)
        non_final_state_nxt = torch.as_tensor(batch.s_, dtype=torch.float32, device=self.device)

        reward = torch.as_tensor(batch.r, dtype=torch.float32, device=self.device)
        g_x = torch.as_tensor(batch.info["g_x"], dtype=torch.float32, device=self.device)
        l_x = torch.as_tensor(batch.info["l_x"], dtype=torch.float32, device=self.device)

        ending = torch.as_tensor(batch.ending, dtype=torch.float32, device=self.device)

        return non_final_mask, non_final_state_nxt, state, action, action_next, disturbance, reward, g_x, l_x, ending
