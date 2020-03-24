from gym_aero.envs import env_base
from math import sin, cos, acos, pi
import gym
import numpy as np

class PerchEnv(env_base.AeroEnv):
    def __init__(self):
        super(PerchEnv, self).__init__()
        self.name = "Perch-v0"
        
        self.goal_xyz = [2.5, 0, 0]
        self.goal_zeta = [0, pi/2., 0]
        self.goal_uvw = [0, 0, 0]
        self.goal_pqr = [0, 0, 0]
        
        self.max_dist = 5
        self.T = 5
        self.pos_thresh = 0.05
        self.ang_thresh = 5.*pi/180.
        self._max_episode_steps = int(self.T/self.ctrl_dt)

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

    def reset(self):
        state = super(PerchEnv, self).reset()
        xyz, sin_zeta, cos_zeta, uvw, pqr, normalized_rpm = state
        self.set_current_dists((xyz, sin_zeta, cos_zeta, uvw, pqr), self.hov_rpm_, normalized_rpm)
        obs = self.get_state_obs((xyz, sin_zeta, cos_zeta, uvw, pqr), self.hov_rpm_, normalized_rpm)
        self.set_prev_dists((xyz, sin_zeta, cos_zeta, uvw, pqr), self.hov_rpm_, normalized_rpm)
        return obs
    
    def render(self, mode='human', video=False, close=False):
        super(PerchEnv, self).render(mode=mode, close=close)
        self.ani.draw_goal(self.goal_xyz)
        self.ani.draw()
        if video: self.ani.save_frame("Perch")
        if close:
            self.ani.close_window()
            self.init_rendering = False
        
