from gym_aero.envs import env_base
from math import sin, cos, acos, pi
import gym
import numpy as np

class PerchEnv(env_base.AeroEnv):
    def __init__(self):
        super(PerchEnv, self).__init__()
        
        self.goal_xyz = [2.5, 0, 0]
        self.goal_zeta = [0, pi/2., 0]
        self.goal_uvw = [0, 0, 0]
        self.goal_pqr = [0, 0, 0]
        
        self.max_dist = 5
        self.T = 5
        self.pos_thresh = 0.05
        self.ang_thresh = 5.*pi/180.

        self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(20,))
    
    def reward(self, xyz, sin_zeta, cos_zeta, uvw, pqr, action):
        dist_rew = 100.*(self.prev_dist-self.curr_dist)
        att_rew = 10.*((self.prev_att_sin-self.curr_att_sin)+(self.prev_att_cos-self.curr_att_cos))
        vel_rew = 0.1*(self.prev_vel-self.curr_vel)
        ang_rew = 0.1*(self.prev_ang-self.curr_ang)

        ctrl_rew = 0.
        ctrl_rew -= sum([((a-self.hov_rpm)/self.max_rpm)**2 for a in action])
        ctrl_rew -= sum([((a-pa)/self.max_rpm)**2 for a, pa in zip(action, self.prev_action)])
        ctrl_rew -= 10.*sum([(u-v)**2 for u, v in zip(uvw, self.prev_uvw)])
        ctrl_rew -= 10.*sum([(p-q)**2 for p, q in zip(pqr, self.prev_pqr)])

        time_rew = 0.

        complete_rew = 500 if att_rew/10. < self.ang_thresh else 0
        complete_rew = complete_rew + 500 if dist_rew/100. < self.pos_thresh else complete_rew
            
        total_reward = dist_rew+att_rew+vel_rew+ang_rew+ctrl_rew+time_rew+complete_rew
        return total_reward, {"dist_rew": dist_rew, 
                                "att_rew": att_rew, 
                                "vel_rew": vel_rew, 
                                "ang_rew": ang_rew, 
                                "ctrl_rew": ctrl_rew, 
                                "time_rew": time_rew,
                                "complete_rew": complete_rew}

    def terminal(self):
        if self.curr_dist >= self.max_dist:
            return True
        elif self.t*self.ctrl_dt > self.T: 
            return True
        else: 
            return False
    
    def get_state_obs(self, state, action, normalized_rpm):
        xyz, sin_zeta, cos_zeta, uvw, pqr = state
        xyz_obs = [x - g for x, g in zip(xyz, self.goal_xyz)]
        zeta_obs = [sz - sin(g) for sz, g in zip(sin_zeta, self.goal_zeta)]+[cz - cos(g) for cz, g in zip(cos_zeta, self.goal_zeta)]
        vel_obs = [u - g for u, g in zip(uvw, self.goal_uvw)]+[p - g for p, g in zip(pqr, self.goal_pqr)]
        curr_tar_obs = xyz_obs+zeta_obs+vel_obs
        next_state = curr_tar_obs+normalized_rpm+[self.t*self.ctrl_dt]
        return next_state
    
    def set_current_dists(self, state, action, normalized_rpm):
        xyz, sin_zeta, cos_zeta, uvw, pqr = state
        self.curr_dist = sum([(x-g)**2 for x, g in zip(xyz, self.goal_xyz)])**0.5
        self.curr_att_sin = sum([(sz-sin(g))**2 for sz, g in zip(sin_zeta, self.goal_zeta)])**0.5
        self.curr_att_cos = sum([(cz-cos(g))**2 for cz, g in zip(cos_zeta, self.goal_zeta)])**0.5
        self.curr_vel = sum([(x-g)**2 for x, g in zip(uvw, self.goal_uvw)])**0.5
        self.curr_ang = sum([(x-g)**2 for x, g in zip(pqr, self.goal_pqr)])**0.5
    
    def set_prev_dists(self, state, action, normalized_rpm):
        xyz, sin_zeta, cos_zeta, uvw, pqr = state
        self.prev_dist = self.curr_dist
        self.prev_att_sin = self.curr_att_sin
        self.prev_att_cos = self.curr_att_cos
        self.prev_vel = self.curr_vel
        self.prev_ang = self.curr_ang
        self.prev_xyz = xyz
        self.prev_zeta = [acos(z) for z in cos_zeta]
        self.prev_uvw = uvw
        self.prev_pqr = pqr
        self.prev_action = action

    def step(self, action):
        commanded_rpm = self.translate_action(action)
        xyz, zeta, uvw, pqr = super(PerchEnv, self).step(commanded_rpm)
        sin_zeta = [sin(z) for z in zeta]
        cos_zeta = [cos(z) for z in zeta]
        curr_rpm = self.get_rpm()
        normalized_rpm = [rpm/self.max_rpm for rpm in curr_rpm]
        self.set_current_dists((xyz, sin_zeta, cos_zeta, uvw, pqr), commanded_rpm, normalized_rpm)
        reward, info = self.reward(xyz, sin_zeta, cos_zeta, uvw, pqr, commanded_rpm)
        done = self.terminal()
        obs = self.get_state_obs((xyz, sin_zeta, cos_zeta, uvw, pqr), commanded_rpm, normalized_rpm)
        self.set_prev_dists((xyz, sin_zeta, cos_zeta, uvw, pqr), commanded_rpm, normalized_rpm)
        self.t += 1
        return obs, reward, done, info

    def reset(self):
        state = super(PerchEnv, self).reset()
        xyz, sin_zeta, cos_zeta, uvw, pqr, normalized_rpm = state
        self.set_current_dists((xyz, sin_zeta, cos_zeta, uvw, pqr), self.hov_rpm_, normalized_rpm)
        obs = self.get_state_obs((xyz, sin_zeta, cos_zeta, uvw, pqr), self.hov_rpm_, normalized_rpm)
        self.set_prev_dists((xyz, sin_zeta, cos_zeta, uvw, pqr), self.hov_rpm_, normalized_rpm)
        return obs
    
    def render(self, mode='human', close=False):
        super(PerchEnv, self).render(mode=mode, close=close)
        self.ani.draw_goal(self.goal_xyz)
        self.ani.draw()
        if close:
            self.ani.close_window()
            self.init_rendering = False
        
