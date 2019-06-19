import numpy as np
import random
from math import pi, sin, cos, acos, exp, tanh
import gym
from gym import error, spaces, utils
from gym.utils import seeding
from gym_aero.envs import path_follow_env
import simulation.animation_gl as ani_gl

from scipy import interpolate

# ToDo:
# update velocity reward -- try to maintain constant velocity (or constant energy?)
# update angle -- have aircraft turn into the trajectory -- only sample from the forward hemisphere

class TermPathFollowEnv(path_follow_env.PathFollowEnv):
    """
    Environment wrapper for training low-level flying skills. The aim is to sequentially fly to
    two consecutive waypoints that are each uniformly sampled from the volume of a sphere. The
    first sphere is centered on the starting point (0,0,0), and the second sphere is centered on
    the point (xg,yg,zg). The agent is able to see both waypoints.
    
    The aircraft has a deterministic starting state by default.

    -- Sean Morrison
    """

    def __init__(self):
        super(TermPathFollowEnv, self).__init__()

    def term_reward(self, state):
        xyz, _, _, _, _ = state
        u = sum([(x-g)**2 for x, g in zip(xyz, self.goal_xyz)])**0.5
        v = sum([(x-g)**2 for x, g in zip(xyz, self.goal_xyz_next)])**0.5
        return -15.*u**2-u*v
    
    def terminal(self, state, term):
        xyz, zeta, uvw, pqr = state
        sq_err = [(x-g)**2 for x, g in zip(xyz, self.goal_xyz)]
        mag = (sum(sq_err))**0.5
        if term == 1:
            return True
        elif mag >= self.max_dist:
            #print("Max dist exceeded")
            return True
        elif self.t*self.ctrl_dt >= self.T:
            #print("Time exceeded") 
            return True
        else: 
            return False

    def step(self, action, term):
        self.t += 1
        action = self.translate_action(action)
        xyz, zeta, uvw, pqr = super(path_follow_env.PathFollowEnv, self).step(action)
        sin_zeta = [sin(z) for z in zeta]
        cos_zeta = [cos(z) for z in zeta]
        curr_rpm = self.get_rpm()
        normalized_rpm = [rpm/self.max_rpm for rpm in curr_rpm]
        reward, info = self.reward(xyz, sin_zeta, cos_zeta, uvw, pqr, action)
        term_rew = self.term_reward((xyz, sin_zeta, cos_zeta, uvw, pqr)) if term == 1 else 0.
        if term == 1: self.next_goal()
        done = self.terminal((xyz, zeta, uvw, pqr), term)
        obs = self.get_state_obs((xyz, sin_zeta, cos_zeta, uvw, pqr, normalized_rpm))
        info.update({"term_rew" : term_rew})
        return obs, reward, done, info
    
    def next_goal(self):
        if not self.goal >= len(self.goal_list_xyz)-1:
            self.time_state = float(self.T)
            self.t = 0
            self.goal += 1
            self.goal_xyz = self.goal_list_xyz[self.goal]
        if self.goal_next >= len(self.goal_list_xyz)-1:
            self.goal_xyz_next = [0., 0., 0.]
            self.goal_zeta_next = [0., 0., 0.]
        else:
            self.goal_next += 1
            self.goal_xyz_next = self.goal_list_xyz[self.goal_next]