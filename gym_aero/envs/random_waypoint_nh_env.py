from gym_aero.envs import random_waypoint_fh_env
from math import sin, cos, acos, pi
import gym
import numpy as np
import random

class RandomWaypointNHEnv(random_waypoint_fh_env.RandomWaypointFHEnv):
    def __init__(self):
        super(RandomWaypointNHEnv, self).__init__()
        self.name = "RandomWaypointNH-v0"
    
    def reward(self, state, action):
        xyz, sin_zeta, cos_zeta, xyz_dot, pqr = state

        dist_rew = 100 * (self.prev_dist - self.curr_dist)
        att_rew = 10 * ((self.prev_att_sin - self.curr_att_sin) + (self.prev_att_cos - self.curr_att_cos))
        vel_rew = 0.1 * (self.prev_vel - self.curr_vel)
        ang_rew = 0.1 * (self.prev_ang - self.curr_ang)

        ctrl_rew = -1*sum([((a-self.hov_rpm)/self.max_rpm)**2 for a in action])
        ctrl_rew -= 1*sum([((a-pa)/self.max_rpm)**2 for a, pa in zip(action, self.prev_action)])
        ctrl_rew -= 0.5*sum([(u-v)**2 for u, v in zip(xyz_dot, self.prev_xyz_dot)])
        ctrl_rew -= 0.5*sum([(p-q)**2 for p, q in zip(pqr, self.prev_pqr)])
        
        time_rew = 0.

        total_reward = dist_rew+att_rew+vel_rew+ang_rew+ctrl_rew+time_rew

        uvw = self.inertial_to_body(xyz_dot)
        nh_reward = -0.1*(uvw[1])**2 # reward for nonholonomic waypoint navigation
        f_reward = -1 if uvw[0] < 0 else 1
        total_reward += nh_reward + f_reward
        return total_reward, {"dist_rew": dist_rew,
                            "att_rew": att_rew,
                            "vel_rew": vel_rew,
                            "ang_rew": ang_rew,
                            "ctrl_rew": ctrl_rew,
                            "time_rew": time_rew}

    def render(self, mode='human', video=False, close=False):
        super(random_waypoint_fh_env.RandomWaypointFHEnv, self).render(mode=mode, close=close)
        self.ani.draw_goal(self.goal_xyz)
        self.ani.draw_vector(self.goal_xyz, self.goal_zeta)
        self.ani.draw()
        if video: self.ani.save_frame(self.name)
        if close:
            self.ani.close_window()
            self.init_rendering = False

    def reset(self):
        self.goal_zeta = self.generate_heading()
        obs = super(RandomWaypointNHEnv, self).reset()
        return obs