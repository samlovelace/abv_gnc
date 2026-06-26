import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from abv_msgs.srv import AbvControlAction
from abv_msgs.msg import AbvVec3

from SAC import SAC
from config import ceConfig
import pickle
import numpy as np
import torch
import yaml
import os
import random
from types import SimpleNamespace
import gymnasium.spaces
from copy import deepcopy


@dataclass
class ControlContext:
    """Context for control policy computation. Includes current state, goal, and error.
        ROS2 message is converted to this internal type on reciept"""
    
    pose:     list[float]
    velocity: list[float]
    goal:     list[float]
    error:    list[float]
    
class PIDController:
    """Simple PID controller for testing interface to abv_controller software 
        running the feedback control loop"""
        
    def __init__(self):
        
        self._kp = [5.0, 5.0, 5.0]
        self._ki = [0.0, 0.0, 0.0]
        self._kd = [15, 15, 15]

        self._integral   = [0.0, 0.0, 0.0]
        self._prev_error = [0.0, 0.0, 0.0]
        self._prev_time  = None

    def compute(self, ctx: ControlContext, now) -> list[float]:
        if self._prev_time is None:
            self._prev_time = now
            return [0.0, 0.0, 0.0]

        dt = (now - self._prev_time).nanoseconds / 1e9
        self._prev_time = now

        if dt <= 0.0:
            return [0.0, 0.0, 0.0]

        action = [0.0, 0.0, 0.0]
        for i in range(3):
            self._integral[i]  += ctx.error[i] * dt
            derivative          = (ctx.error[i] - self._prev_error[i]) / dt
            action[i]           = (self._kp[i] * ctx.error[i] +
                                   self._ki[i] * self._integral[i] +
                                   self._kd[i] * derivative)
            
            self._prev_error[i] = ctx.error[i]

        return action

    def reset(self):
        self._integral   = [0.0, 0.0, 0.0]
        self._prev_error = [0.0, 0.0, 0.0]
        self._prev_time  = None
       
class RLController:

    def __init__(self):
        
        with open("config.yaml", "r") as f:
            config_dict = yaml.safe_load(f)
        
        args = SimpleNamespace(**{k: v for section in config_dict.values() for k, v in section.items()})
        self.device       = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        if args.annealing:
            GAMMA_END        = 0.999999
            EPS_PERIOD       = int(args.updatePeriodEps / 10)
            EPS_RESET_PERIOD = args.updatePeriodEps
        else:
            GAMMA_END        = 0.999
            EPS_PERIOD       = args.updatePeriodEps
            EPS_RESET_PERIOD = args.maxUpdates
        sample_inside_obs = False
    
        self.stateDim    = 6 + 3 * args.numObstacles
        self.actionNum   = 3

        config_params = {
            "DEVICE": self.device,
            "ENV_NAME": "cont-obs-avoid-v0",
            "NUM_ENVS": args.numEnvs,
            "SEED": args.randomSeed,
            "MAX_UPDATES": args.maxUpdates,
            "MAX_EP_STEPS": args.maxSteps,
            "BATCH_SIZE": args.batchSize,
            "MEMORY_CAPACITY": args.memoryCapacity,
            "C_ARCHITECTURE": args.criticArchitecture,
            "A_ARCHITECTURE": args.actorArchitecture,
            "ACTIVATION": args.actType,
            "GAMMA": args.gamma,
            "GAMMA_PERIOD": args.updatePeriodGamma,
            "GAMMA_END": GAMMA_END,
            "GAMMA_DECAY": args.gammaDecay,
            "EPSILON": args.eps,
            "EPS_END": args.epsEnd,
            "EPS_PERIOD": EPS_PERIOD,
            "EPS_DECAY": args.epsDecay,
            "EPS_RESET_PERIOD": EPS_RESET_PERIOD,
            "LR_C": args.learningRate,
            "LR_C_END": args.learningRate * 0.5,
            "LR_C_PERIOD": args.updatePeriodLr,
            "LR_C_DECAY": args.learningRateDecay,
            "LR_A": args.learningRateActor,
            "LR_A_END": args.learningRateActor * 0.5,
            "LR_A_PERIOD": args.updatePeriodLr,
            "LR_A_DECAY": args.learningRateDecay,
            "MAX_MODEL": args.maxModel,
            "NUM_CRITICS": args.numCritics,
            "SELECT_WORST_Q": args.selectWorstQ,
            "FIND_MAX_Q": args.findMaxQ,
            "SIM_MAX_Q": args.simMaxQ,
            "TIME_STEP": args.timeStep,
            "TAU": args.tau,
            "HARD_UPDATE": args.hardUpdate,
            "SOFT_UPDATE": args.softUpdate,
            "RENDER": args.render,
            "DOUBLE": args.double,
            "REWARD": args.reward,
            "PENALTY": args.penalty,
            "ALPHA": args.alpha,
            "POLICY": args.policy,
            "TARGET_UPDATE_INTERVAL": args.targetUpdateInterval,
            "AUTO_ALPHA_TUNING": args.autoAlphaTuning
        }
        CONFIG = ceConfig(**config_params)

        action_space = gymnasium.spaces.Box(low=-np.ones((self.actionNum,), dtype=np.float32), high=np.ones((self.actionNum,), dtype=np.float32), shape=(self.actionNum,))
        disturbance_space = deepcopy(action_space)

        c_dimList     = [self.stateDim] + CONFIG.C_ARCHITECTURE + [self.actionNum]
        a_dimList     = [self.stateDim] + CONFIG.A_ARCHITECTURE + [self.actionNum]
        self.sacAgent = SAC(CONFIG, c_dimList=c_dimList, a_dimList=a_dimList, action_space=action_space, disturbance_space=disturbance_space)

        best_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "best_models")
        self.sacAgent.load_best_models(best_dir, evaluate=True)

        # self.force_scale

        random.seed(args.randomSeed)
        self.obstacles = []
        x_lim = np.array([-0.5, 4])
        y_lim = np.array([-0.5, 2.0])
        for i in range(args.numObstacles):
            center_x = random.random() * (x_lim[1] - x_lim[0]) + x_lim[0]
            center_y = random.random() * (y_lim[1] - y_lim[0]) + y_lim[0]
            radius = 0.1
            self.obstacles.append((np.array([center_x, center_y]), radius))

    def calculate_margin_circle(s, c_r, negativeInside=True):
        """Calculates the margin to a circle in the x-y state space.

            Args:
                s (np.ndarray): the state of the agent. It requires that s[0] is the
                    x position and s[1] is the y position.
                c_r (tuple of np.ndarray and float)): (center, radius).
                negativeInside (bool, optional): add a negative sign to the distance
                    if inside the circle. Defaults to True.

            Returns:
                float: margin.
            """
        center, radius = c_r
        dist_to_center = np.linalg.norm(s - center, axis=1)
        dir_x = (s[:, 0] - center[0]) / (dist_to_center + 1.0e-10)
        dir_y = (s[:, 1] - center[1]) / (dist_to_center + 1.0e-10)
        margin = dist_to_center - radius

        if negativeInside:
            return dir_x, dir_y, margin
        else:
            return dir_x, dir_y, -margin

    def load_obj(filename):
        """Loads the object and return the object.

        Args:
            filename (str): the path to save the object.
        """
        with open(filename + ".pkl", "rb") as f:
            return pickle.load(f)

    def compute(self, ctx: ControlContext, now) -> list[float]: 

        rel_x = ctx.error[0]
        rel_y = ctx.error[1]
        theta = ctx.pose[2]
        vx = ctx.velocity[0]
        vy = ctx.velocity[1]
        omega = ctx.velocity[2]

        observations_np = np.column_stack([rel_x, rel_y, theta, vx, vy, omega]) # relative position + theta + vx, vy, omega
        for constraint_set in self.obstacles:
            dir_x, dir_y, g_x_i = self.calculate_margin_circle(ctx.pose[None, :2], constraint_set, negativeInside=False)
            observations_np = np.column_stack([observations_np, dir_x, dir_y, g_x_i])

        observations = torch.FloatTensor(observations_np).to(self.device)
        
        _, _, action =  self.sacAgent.protagonist.sample(observations) # deterministic action
        _, _, disturb = self.sacAgent.adversary.sample(observations) # deterministic disturbance

        q_val = self.sacAgent.Q_network(observations, action, disturb)

        l_x = self.target_margin(ctx.pose[:2])
        g_x = self.safety_margin(ctx.pose[:2])
        value = max(l_x, g_x)

        return [action[0], action[1], action[2]]

class RLPolicyNode(Node):
    def __init__(self):
        super().__init__('abv_rl')

        self._controller = PIDController()

        self.create_service(
            AbvControlAction,
            'abv/control_action',
            self._handle_request
        )

        self.get_logger().info('Policy node ready')

    def _handle_request(self, request, response):
        
        ctx = ControlContext(
            pose     = [request.pose.x, request.pose.y, request.pose.yaw],
            velocity = [request.vel.x, request.vel.y, request.vel.yaw],
            goal     = [request.goal.x, request.goal.y, request.goal.yaw],
            error    = [request.error.x, request.error.y, request.error.yaw]
        )
        
        action = self._controller.compute(ctx, self.get_clock().now())

        response.action.x   = action[0]
        response.action.y   = action[1]
        response.action.yaw = action[2]
        
        response.is_global = False

        return response


def main():
    rclpy.init()
    node = RLPolicyNode()
    rclpy.spin(node)
    rclpy.shutdown()