"""
Please contact the author(s) of this library if you have any questions.
Authors: Kai-Chieh Hsu ( kaichieh@princeton.edu )

This module implements a Sin activation function and neural network model as
torch.nn.Module. Also, it implements a scheduler for hyper-parameters.
"""

import abc
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Normal


LOG_SIG_MAX = 1
LOG_SIG_MIN = -5
epsilon = 1e-6

# Initialize Policy weights
def weights_init_(m):
    if isinstance(m, nn.Linear):
        torch.nn.init.xavier_uniform_(m.weight, gain=1)
        torch.nn.init.constant_(m.bias, 0)

# == Scheduler ==
class _scheduler(abc.ABC):
  """
  The parent class for schedulers. It implements some basic functions that will
  be used in all scheduler.
  """

  def __init__(self, last_epoch=-1, verbose=False):
    """Initializes the scheduler with the index of last epoch.
    """
    self.cnt = last_epoch
    self.verbose = verbose
    self.variable = None
    self.step()

  def step(self):
    """Updates the index of the last epoch and the variable.
    """
    self.cnt += 1
    value = self.get_value()
    self.variable = value

  @abc.abstractmethod
  def get_value(self):
    raise NotImplementedError

  def get_variable(self):
    """Returns the variable.
    """
    return self.variable


class StepLR(_scheduler):
  """This scheduler will decay to end value periodically.
  """

  def __init__(
      self, initValue, period, decay=0.1, endValue=0., last_epoch=-1,
      verbose=False
  ):
    """Initializes an object of the scheduler with the specified attributes.

    Args:
        initValue (float): initial value of the variable.
        period (int): the period to update the variable.
        decay (float, optional): the amount by which the variable decays.
            Defaults to 0.1.
        endValue (float, optional): the target value to decay to.
            Defaults to 0.
        last_epoch (int, optional): the index of the last epoch.
            Defaults to -1.
        verbose (bool, optional): print messages if True. Defaults to False.
    """
    self.initValue = initValue
    self.period = period
    self.decay = decay
    self.endValue = endValue
    super(StepLR, self).__init__(last_epoch, verbose)

  def get_value(self):
    """Returns the value of the variable.
    """
    if self.cnt == -1:
      return self.initValue

    numDecay = int(self.cnt / self.period)
    tmpValue = self.initValue * (self.decay**numDecay)
    if self.endValue is not None and tmpValue <= self.endValue:
      return self.endValue
    return tmpValue
  
class Sin(nn.Module):
  """An element-wise sin activation wrapped as a nn.Module.

  Shape:
      - Input: `(N, *)` where `*` means, any number of additional dimensions
      - Output: `(N, *)`, same shape as the input

  Examples:
      >>> m = Sin()
      >>> input = torch.randn(2)
      >>> output = m(input)
  """

  def forward(self, input):
    return torch.sin(input)  # simply apply already implemented sin


class StepLRMargin(_scheduler):

  def __init__(
      self, initValue, period, goalValue, decay=0.1, endValue=1, last_epoch=-1, numEnvs=1,
      verbose=False
  ):
    """Initializes an object of the scheduler with the specified attributes.

    Args:
        initValue (float): initial value of the variable.
        period (int): the period to update the variable.
        goalValue (float):the target value to anneal to.
        decay (float, optional): the amount by which the margin between the
            variable and the goal value decays. Defaults to 0.1.
        endValue (float, optional): the maximum value of the variable.
            Defaults to 1.
        last_epoch (int, optional): the index of the last epoch.
            Defaults to -1.
        verbose (bool, optional): print messages if True. Defaults to False.
    """
    self.initValue = initValue
    self.period = period
    self.decay = decay
    self.endValue = endValue
    self.goalValue = goalValue
    self.numEnvs = numEnvs
    super(StepLRMargin, self).__init__(last_epoch, verbose)

  def get_value(self):
    """Returns the value of the variable.
    """
    if self.cnt == -1:
      return self.initValue

    numDecay = int((self.cnt * self.numEnvs) / self.period)
    # if numDecay > 0:
    #  print('gamma update')
    tmpValue = self.goalValue - (self.goalValue
                                 - self.initValue) * (self.decay**numDecay)
    if self.endValue is not None and tmpValue >= self.endValue:
      return self.endValue
    return tmpValue

# ==== SAC Networks ====

def build_mlp(dimList, activation, layernorm=False):
    act_map = {
        "Sin":  Sin(),
        "Tanh": nn.Tanh(),
        "ReLU": nn.ReLU(),
    }
    if activation not in act_map:
        raise ValueError(f"Activation type ({activation}) is not included!")

    layers = []
    numLayer = len(dimList) - 1
    for idx in range(numLayer):
        layers.append(nn.Linear(dimList[idx], dimList[idx + 1]))
        if layernorm:
            layers.append(nn.LayerNorm(dimList[idx + 1]))
        if idx < numLayer - 1:          # no activation after final layer
            layers.append(act_map[activation])

    return nn.Sequential(*layers)


class QNetwork(nn.Module):
    def __init__(self, CONFIG, dimList, numActions, numDisturb):
        super(QNetwork, self).__init__()

        self.config = CONFIG
        self.actType = CONFIG.ACTIVATION
        sa_dimList = dimList.copy()
        sa_dimList[0] = dimList[0] + numActions + numDisturb  # input is state-action pair
        sa_dimList[-1] = 1  # output is Q-value
        print("QNetwork dimList:", sa_dimList)

        self.q_head1 = build_mlp(sa_dimList, self.actType, layernorm=False)
        self.q_head2 = build_mlp(sa_dimList, self.actType, layernorm=False)

        self.apply(weights_init_)

    def forward(self, state, action, disturbance):
        xu = torch.cat([state, action, disturbance], 1)

        x1 = self.q_head1(xu)
        x2 = self.q_head2(xu)

        return x1, x2


class GaussianPolicy(nn.Module):
    def __init__(self, CONFIG, dimList, numActions, action_space=None, conditioned_sigma=False):
        super(GaussianPolicy, self).__init__()
        
        self.config = CONFIG
        self.actType = CONFIG.ACTIVATION
        self.c_sigma = conditioned_sigma
        print("GaussianPolicy dimList:", dimList)

        self.mu_head = build_mlp(dimList, self.actType)
        log_std_dimList = dimList.copy()
        log_std_dimList[-1] = numActions
        if self.c_sigma:
            self.log_std_head = build_mlp(log_std_dimList, self.actType)
        else:
            self.log_std_head = nn.Parameter(torch.zeros(numActions))

        self.apply(weights_init_)

        # action rescaling
        if action_space is None:
            self.action_scale = torch.tensor(1.)
            self.action_bias = torch.tensor(0.)
        else:
            self.action_scale = torch.FloatTensor(
                (action_space.high - action_space.low) / 2.)
            self.action_bias = torch.FloatTensor(
                (action_space.high + action_space.low) / 2.)

    def forward(self, state):
        mean = self.mu_head(state)
        if self.c_sigma:
            log_std = self.log_std_head(state)
            sigma = torch.clamp(log_std, min=LOG_SIG_MIN, max=LOG_SIG_MAX)
        else:
            shape = [1] * len(mean.shape)
            shape[1] = -1
            sigma = (self.log_std_head.view(shape) + torch.zeros_like(mean)).exp()
        return mean, sigma

    def sample(self, state):
        mean, std = self.forward(state)
        if self.c_sigma:
            std = std.exp()
        normal = Normal(mean, std)
        x_t = normal.rsample()  # for reparameterization trick (mean + std * N(0,1))
        y_t = torch.tanh(x_t)
        action = y_t * self.action_scale + self.action_bias
        log_prob = normal.log_prob(x_t)
        # Enforcing Action Bound
        log_prob -= (2 * (torch.log(torch.tensor(2)) - x_t - F.softplus(-2 * x_t)))
        log_prob = log_prob.sum(-1, keepdim=True)
        # log_prob = log_prob.sum(1, keepdim=True)
        mean = torch.tanh(mean) * self.action_scale + self.action_bias
        return action, log_prob, mean

    def to(self, device):
        self.action_scale = self.action_scale.to(device)
        self.action_bias = self.action_bias.to(device)
        return super(GaussianPolicy, self).to(device)