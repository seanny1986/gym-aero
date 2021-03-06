import numpy as np
import random
from math import pi, sin, cos
import gym
from gym import error, spaces, utils
from gym.utils import seeding
from gym_aero.envs import land_env
import simulation.animation_gl as ani_gl


class TermLandEnv(land_env.LandEnv):
    """
        Environment wrapper for training low-level flying skills. The aircraft is required to land
        at a random positon. It is required to lower onto the landing pad, i.e the angle
        between the aircraft and pad is 90 deg.
    """
    def __init__(self):
        super(TermLandEnv, self).__init__()

    def term_reward(self, state):
        xyz, _, _, _, _ = state
        u = sum([(x-g)**2 for x, g in zip(xyz, self.goal_xyz)])**0.5
        return -15.*u**2-u
    
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
        xyz, zeta, uvw, pqr = super(land_env.LandEnv, self).step(action)
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
        pass