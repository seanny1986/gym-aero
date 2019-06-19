import math
import torch
import torch.nn as nn
import torch.nn.functional as F
import matplotlib.pyplot as plt
import matplotlib.style as style
import pandas as pd
import random
import numpy as np
from collections import namedtuple

"""
    This module contains utility functions and classes that would clutter up other scripts. These include getting
    trajectory information from the dataset, doing Euler angle quaternion conversions, saving and loading models,
    and logging functionality. If you need some kind of visualization of utility function to be implemented and it
    doesn't make sense for it to go anywhere else, this is where it belongs.
"""

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def numpy_to_pytorch(xyz, zeta, uvw, pqr, cuda=True):
    xyz = torch.from_numpy(xyz.T).float()
    zeta = torch.from_numpy(zeta.T).float()
    uvw = torch.from_numpy(uvw.T).float()
    pqr = torch.from_numpy(pqr.T).float()
    if cuda:
        xyz = xyz.cuda()
        zeta = zeta.cuda()
        uvw = uvw.cuda()
        pqr = pqr.cuda()
    return xyz, zeta, uvw, pqr

def average_gradient(model):
    mean = []
    for param in model.parameters():
        mean.append(param.grad.mean().item())
    return float(sum(mean))/float(len(mean))

def print_gradients(model):
    for param in model.parameters():
        print(param.grad)

def save(model, filename):
    print("=> Saving model in '{}'".format(filename))
    torch.save(model, filename)

def load(filename):
    print("=> Loading '{}'".format(filename))
    return torch.load(filename, map_location=lambda storage, loc: storage)

def progress(count, total, loss):
    bar_len = 50
    filled_len = int(round(bar_len * count / float(total)))
    percent = round(100.0 * count / float(total), 1)
    loss = tuple([round(x, 5) if isinstance(x, float) else x for x in loss])
    bar = '#' * filled_len + '-' * (bar_len - filled_len)
    print('[{}] {}%, Loss: {}'.format(bar, percent, loss), end='\r', flush=True)

def set_seed(seed):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)

def cuda_if(torch_object, cuda):
    return torch_object.cuda() if cuda else torch_object

def set_lr(optimizer, lr):
    for param_group in optimizer.param_groups:
        param_group['lr'] = lr

class RunningStat(object):
    def __init__(self, shape):
        self._n = 0
        self._M = np.zeros(shape)
        self._S = np.zeros(shape)

    def push(self, x):
        x = np.asarray(x)
        assert x.shape == self._M.shape
        self._n += 1
        if self._n == 1:
            self._M[...] = x
        else:
            oldM = self._M.copy()
            self._M[...] = oldM+(x-oldM)/self._n
            self._S[...] = self._S+(x-oldM)*(x-self._M)

    @property
    def n(self):
        return self._n

    @property
    def mean(self):
        return self._M

    @property
    def var(self):
        return self._S/(self._n-1) if self._n>1 else np.square(self._M)

    @property
    def std(self):
        return np.sqrt(self.var)

    @property
    def shape(self):
        return self._M.shape

class ZFilter:
    """
    y = (x-mean)/std
    using running estimates of mean,std
    """

    def __init__(self, shape, demean=True, destd=True, clip=10.0):
        self.demean = demean
        self.destd = destd
        self.clip = clip
        self.rs = RunningStat(shape)

    def __call__(self, x, update=True):
        if update: self.rs.push(x)
        if self.demean:
            x = x-self.rs.mean
        if self.destd:
            x = x/(self.rs.std+1e-8)
        if self.clip:
            x = np.clip(x, -self.clip, self.clip)
        return x

    def output_shape(self, input_space):
        return input_space.shape

# from https://github.com/songrotek/DDPG/blob/master/ou_noise.py
class OUNoise:
    def __init__(self, action_dimension, scale=0.1, mu=0, theta=0.15, sigma=0.2):
        self.action_dimension = action_dimension
        self.scale = scale
        self.mu = mu
        self.theta = theta
        self.sigma = sigma
        self.state = np.ones(self.action_dimension)*self.mu
        self.reset()
        self.alpha = 0.01

    def reset(self):
        self.state = np.ones(self.action_dimension)*self.mu

    def noise(self):
        x = self.state
        dx = self.theta*(self.mu-x)+self.sigma*np.random.randn(len(x))
        self.state = x+dx
        return self.state*self.scale

    def set_seed(self,seed):
        np.random.seed(seed=seed)
    
    def anneal(self):
        # annealing the exploration noise by progressively stepping mu and sigma to 0. The reason
        # for returning the mean and sigma is so that I can check the determinism of the noise that
        # is being injected. If the noise is within a certain threshold, we probably don't want use 
        # it, and instead let the policy act deterministically.
        
        if abs(self.mu) > 0:
            d_mu = 0-self.mu
            d_sig = 0-self.sigma
            self.mu += self.alpha*d_mu
            self.sigma += self.alpha*d_sig
        return self.mu, self.sigma

Transition = namedtuple('Transition', ['state', 'action', 'next_state', 'reward'])
class ReplayMemory:
    def __init__(self, capacity):
        self.capacity = capacity
        self.memory = []
        self.position = 0

    def push(self, *args):
        if len(self.memory) < self.capacity:
            self.memory.append(None)
        self.memory[self.position] = Transition(*args)
        self.position = (self.position+1)%self.capacity

    def sample(self, batch_size):
        if self.__len__() < batch_size:
            return self.memory
        else:
            return random.sample(self.memory, batch_size)

    def __len__(self):
        return len(self.memory)

class Actor(torch.nn.Module):
    def __init__(self, input_dim, hidden_dim, output_dim):
        super(Actor,self).__init__()
        self.__input_dim = input_dim
        self.__hidden_dim = hidden_dim
        self.__output_dim = output_dim

        self.__l1 = torch.nn.Linear(input_dim, hidden_dim)
        self.__mu = torch.nn.Linear(hidden_dim, output_dim)
        self.__logvar = torch.nn.Linear(hidden_dim, output_dim)
        self.__mu.weight.data.mul_(0.1)
        self.__mu.bias.data.mul_(0.0)
        self.__logvar.weight.data.mul_(0.1)
        self.__logvar.bias.data.mul_(0.0)

    def forward(self, x):
        x = F.tanh(self.__l1(x))
        mu = self.__mu(x)
        logvar = self.__logvar(x)
        return mu, logvar

class ActorLSTM(torch.nn.Module):
    def __init__(self, input_dim, hidden_dim, output_dim):
        super(ActorLSTM,self).__init__()
        self.__input_dim = input_dim
        self.__hidden_dim = hidden_dim
        self.__output_dim = output_dim

        self.__l1 = torch.nn.LSTM(input_dim, hidden_dim)
        self.__mu = torch.nn.Linear(hidden_dim, output_dim)
        self.__logvar = torch.nn.Linear(hidden_dim, output_dim)

    def forward(self, inputs):
        x, hidden = inputs
        x = x.view(x.size(0), -1)
        x, hidden = self.__l1(x, hidden)
        mu = self.__mu(x)
        logvar = self.__logvar(x)
        return mu, logvar, hidden

class Critic(nn.Module):
    def __init__(self, input_dim, hidden_dim, output_dim):
        super(Critic, self).__init__()
        self.__affine1 = nn.Linear(input_dim, hidden_dim)
        self.__value_head = nn.Linear(hidden_dim, output_dim)
        self.__value_head.weight.data.mul_(0.1)
        self.__value_head.bias.data.mul_(0.0)
        

    def forward(self, x):
        x = F.relu(self.__affine1(x))
        q = self.__value_head(x)
        return q
    
