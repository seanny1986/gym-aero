from math import cos, pi, sqrt
import random
import config as cfg
import numpy as np

# Discrete Wind Gust Model
#
# Adapted to python, original model can be found here:
# https://au.mathworks.com/help/aeroblks/discretewindgustmodel.html

class WindModel:
    def __init__(self, mu_u=1., mu_v=0., mu_w=0.):
        self.__gusts = []
        self.energy = 0.
        self.energy_thresh = 10.
        self.dt = cfg.params["dt"]
        self.t = 0
        self.prev_gust = None

    def step(self, xyz):
        wind = 0
        for g in self.__gusts:
            g.dist_travelled += g.uvw*self.dt
            if np.sum(g.dist_travelled >= 6) > 0:
                self.__gusts.pop()
            wind += g.apply_wind(xyz)
        delta_energy = random.random()
        self.energy += delta_energy
        if self.energy >= self.energy_thresh:
            if self.prev_gust is not None:
                gust = DiscreteGust(self.dt, self.prev_gust.uvw)
            else:
                gust = DiscreteGust(self.dt, np.zeros((3,1)))
            self.__gusts.append(gust)
            self.prev_gust = gust
            self.t = 0
        else:
            self.t += self.dt
        return wind

class DiscreteGust:
    def __init__(self, dt, prev_gust, gust_amplitude=1., gust_length=3.):
        dW = sqrt(dt)*np.random.normal(size=(3,1))
        self.uvw = prev_gust+dW
        self.dist_travelled = np.zeros((3,1))
        self.gust_amplitude = gust_amplitude
        self.gust_length = gust_length

    def apply_wind(self, xyz, bound):
        """
    
        """
        if self.dist_travelled < 0:
            wind_res = 0
        elif self.dist_travelled < self.gust_length:
            wind_res = (self.gust_amplitude / 2) * (1.0 - cos((pi * self.dist_travelled) / self.gust_length))
        else:
            wind_res = self.gust_amplitude
        return wind_res