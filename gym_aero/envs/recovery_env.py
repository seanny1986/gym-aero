from gym_aero.envs import env_base
from math import sin, cos, pi, acos
import gym
import numpy as np
import random

class RecoveryEnv(env_base.AeroEnv):
    def __init__(self):
        super(RecoveryEnv, self).__init__()
        self.name = "Recovery-v0"
        
        self.goal_xyz = [0, 0, 0]
        self.goal_zeta = [0, 0, 0]
        self.goal_uvw = [0, 0, 0]
        self.goal_pqr = [0, 0, 0]
        
        self.start_radius = 1.5
        self.start_max_zeta = pi/2
        self.start_max_uvw = 1.5
        self.start_max_pqr = 0.1

        self.max_dist = 5
        self.T = 15
        self._max_episode_steps = int(self.T/self.ctrl_dt)

        self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(20,))
    
    def reward(self, state, action, normalized_rpm):
        xyz, sin_zeta, cos_zeta, uvw, pqr = state
        
        dist_rew = 100*(self.prev_dist-self.curr_dist)
        att_rew = 10*((self.prev_att_sin-self.curr_att_sin)+(self.prev_att_cos-self.curr_att_cos))
        vel_rew = 0.1*(self.prev_vel-self.curr_vel)
        ang_rew = 0.1*(self.prev_ang-self.curr_ang)

        ctrl_rew = -sum([((a-self.hov_rpm)/self.max_rpm)**2 for a in action])
        ctrl_rew -= sum([((a-pa)/self.max_rpm)**2 for a, pa in zip(action, self.prev_action)])
        ctrl_rew -= 10*sum([(x-y)**2 for x, y in zip(xyz, self.prev_xyz)])
        ctrl_rew -= 10*sum([(z-sin(k))**2 for z, k in zip(sin_zeta, self.prev_zeta)])
        ctrl_rew -= 10*sum([(z-cos(k))**2 for z, k in zip(cos_zeta, self.prev_zeta)])
        ctrl_rew -= 10*sum([(u-v)**2 for u, v in zip(uvw, self.prev_uvw)])
        ctrl_rew -= 10*sum([(p-q)**2 for p, q in zip(pqr, self.prev_pqr)])

        time_rew = 10.

        total_reward = dist_rew+att_rew+vel_rew+ang_rew+ctrl_rew+time_rew
        return total_reward, {"dist_rew": dist_rew, 
                                "att_rew": att_rew, 
                                "vel_rew": vel_rew, 
                                "ang_rew": ang_rew, 
                                "ctrl_rew": ctrl_rew, 
                                "time_rew": time_rew}

    def reset(self):
        xyz, zeta, pqr, uvw = self.generate_random_state()
        state = super(RecoveryEnv, self).reset_to_custom_state(xyz, zeta, pqr, uvw, self.hov_rpm_)
        xyz, sin_zeta, cos_zeta, uvw, pqr, normalized_rpm = state
        self.set_current_dists((xyz, sin_zeta, cos_zeta, uvw, pqr), self.hov_rpm_, normalized_rpm)
        obs = self.get_state_obs((xyz, sin_zeta, cos_zeta, uvw, pqr), self.hov_rpm_, normalized_rpm)
        self.set_prev_dists((xyz, sin_zeta, cos_zeta, uvw, pqr), self.hov_rpm_, normalized_rpm)
        return obs
    
    def generate_random_state(self):
        mag = 10.
        while mag > self.start_radius:
            xyz = [random.uniform(-self.start_radius, self.start_radius) for _ in range(3)]
            mag = sum([x**2 for x in xyz])**0.5
        
        zeta = [random.uniform(-self.start_max_zeta, self.start_max_zeta) for _ in range(3)]
        
        mag = 10.
        while mag > self.start_max_uvw:
            uvw = [random.uniform(-self.start_max_uvw, self.start_max_uvw) for _ in range(3)]
            mag = sum([x**2 for x in uvw])**0.5

        mag = 10.
        while mag > self.start_max_pqr:
            pqr = [random.uniform(-self.start_max_pqr, self.start_max_pqr) for _ in range(3)]
            mag = sum([x**2 for x in pqr])**0.5

        return xyz, zeta, pqr, uvw
    
    def render(self, mode='human', video=False, close=False):
        super(RecoveryEnv, self).render(mode=mode, close=close)
        self.ani.draw_goal(self.goal_xyz)
        self.ani.draw()
        if video: self.ani.save_frame("Recovery")
        if close:
            self.ani.close_window()
            self.init_rendering = False
