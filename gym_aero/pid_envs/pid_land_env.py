from gym_aero.envs import land_env
from gym_aero.pid_envs import pid_env_base
from math import sin, cos, acos
import gym
import numpy as np

class PIDLandEnv(land_env.LandEnv, pid_env_base.PIDEnv):
    def __init__(self):
        super(PIDLandEnv, self).__init__()
        self.name = "Land-v0"
        
    def step(self, errors):
        xyz, zeta, uvw, pqr = super(PIDLandEnv, self).pid_step(errors)
        xyz_dot = self.get_xyz_dot()
        sin_zeta = [sin(z) for z in zeta]
        cos_zeta = [cos(z) for z in zeta]
        curr_rpm = self.get_rpm()
        normalized_rpm = [rpm/self.max_rpm for rpm in curr_rpm]
        self.set_current_dists((xyz, sin_zeta, cos_zeta, xyz_dot, pqr), errors)
        reward, info = self.reward((xyz, sin_zeta, cos_zeta, xyz_dot, pqr), errors)
        self.t += 1
        done = self.terminal((xyz, zeta, xyz_dot, pqr))
        obs = self.get_state_obs((xyz, sin_zeta, cos_zeta, xyz_dot, pqr), errors, normalized_rpm)
        self.set_prev_dists((xyz, sin_zeta, cos_zeta, xyz_dot, pqr), errors)
        return obs, reward, done, info