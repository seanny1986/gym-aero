from gym_aero.envs import random_waypoint_nh_env
from gym_aero.pid_envs import pid_env_base
from math import sin, cos, acos
import gym
import numpy as np

class PIDRandomWaypointNHEnv(random_waypoint_nh_env.RandomWaypointNHEnv, pid_env_base.PIDEnv):
    def __init__(self):
        super(PIDRandomWaypointNHEnv, self).__init__()
        self.name = "PIDRandomWaypointNH-v0"
        
    def step(self, errors):
        xyz, zeta, uvw, pqr = super(PIDRandomWaypointNHEnv, self).pid_step(errors)
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
    
    def render(self, mode='human', video=False, close=False):
        super(pid_env_base.PIDEnv, self).render(mode=mode, close=close)
        self.ani.draw_goal(self.goal_xyz)
        self.ani.draw_vector(self.goal_xyz, self.goal_zeta)
        self.ani.draw()
        if video: self.ani.save_frame(self.name)
        if close:
            self.ani.close_window()
            self.init_rendering = False