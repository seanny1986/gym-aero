from gym_aero.comparison import env_base
from math import sin, cos, acos
import gym
import numpy as np

class HoverComparison(env_base.AeroEnv):
    def __init__(self):
        super(HoverComparison, self).__init__()
        
        self.goal_xyz = [0., 0., 0.]
        self.goal_zeta = [0., 0., 0.]
        self.goal_xyz_dot = [0., 0., 0.]
        self.goal_pqr = [0., 0., 0.]
        
        self.max_dist = 5
        self.T = 15
        self._max_episode_steps = int(self.T/self.ctrl_dt)

        self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(35,))
    
    def reward(self, state, action):
        xyz, sin_zeta, cos_zeta, xyz_dot, pqr = state
        
        # agent gets a negative reward based on how far away it is from the desired goal state
        dist_rew = 100 * (self.prev_dist - self.curr_dist)
        att_rew = 10 * ((self.prev_att_sin - self.curr_att_sin) + (self.prev_att_cos - self.curr_att_cos))
        vel_rew = 0.1 * (self.prev_vel - self.curr_vel)
        ang_rew = 0.1 * (self.prev_ang - self.curr_ang)

        # agent gets a negative reward for excessive action inputs
        #ctrl_rew = 0.
        #print("new action: ", action)
        ctrl_rew = -1*sum([((a-self.hov_omega)/self.max_omega)**2 for a in action])
        ctrl_rew -= 1*sum([((a-pa)/self.max_omega)**2 for a, pa in zip(action, self.prev_action)])
        #ctrl_rew -= 1*sum([(x-y)**2 for x, y in zip(xyz, self.prev_xyz)])
        #ctrl_rew -= 1*sum([(z-sin(k))**2 for z, k in zip(sin_zeta, self.prev_zeta)])
        #ctrl_rew -= 1*sum([(z-cos(k))**2 for z, k in zip(cos_zeta, self.prev_zeta)])
        #ctrl_rew -= 10*sum([(u-v)**2 for u, v in zip(xyz_dot, self.prev_xyz_dot)])
        #ctrl_rew -= 10*sum([(p-q)**2 for p, q in zip(pqr, self.prev_pqr)])
        
        # agent gets a positive reward for time spent in flight
        time_rew = 10.

        total_reward = dist_rew+att_rew+vel_rew+ang_rew+ctrl_rew+time_rew

        return total_reward, {"dist_rew": dist_rew, 
                                "att_rew": att_rew,
                                "vel_rew": vel_rew, 
                                "ang_rew": ang_rew, 
                                "ctrl_rew": ctrl_rew,
                                "time_rew": time_rew}

    def reset(self):
        self.t = 0.
        state = super(HoverComparison, self).reset()
        #print(state)
        xyz, sin_zeta, cos_zeta, uvw, pqr, normalized_omega = state
        xyz_dot = self.get_xyz_dot()
        #print(xyz_dot)
        self.set_current_dists((xyz, sin_zeta, cos_zeta, xyz_dot, pqr), self.hov_omega_)
        obs = self.get_state_obs((xyz, sin_zeta, cos_zeta, xyz_dot, pqr), self.hov_omega_, normalized_omega)
        #print(obs)
        self.set_prev_dists((xyz, sin_zeta, cos_zeta, xyz_dot, pqr), self.hov_omega_)
        return obs
    
    def render(self, mode='human', close=False):
        super(HoverComparison, self).render(mode=mode, close=close)
        self.ani.draw_goal(self.goal_xyz)
        self.ani.draw()
        if close:
            self.ani.close_window()
            self.init_rendering = False

