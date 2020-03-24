from gym_aero.envs import env_base
from math import sin, cos, acos, pi
import gym
import numpy as np
import random

class RandomWaypointFHEnv(env_base.AeroEnv):
    def __init__(self):
        super(RandomWaypointFHEnv, self).__init__()
        self.name = "RandomWaypointFH-v0"
        
        self.goal_dist = 1.5
        self.max_dist = 5
        self.T = 5
        self._max_episode_steps = int(self.T/self.ctrl_dt)

        self.goal_xyz_dot = [0., 0., 0.]
        self.goal_pqr = [0., 0., 0.]
        self.goal_zeta = [0., 0., 0.]

        self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(20,))
    
    def reward(self, state, action):
        xyz, sin_zeta, cos_zeta, xyz_dot, pqr = state

        dist_rew = 100 * (self.prev_dist - self.curr_dist)
        att_rew = 10 * ((self.prev_att_sin - self.curr_att_sin) + (self.prev_att_cos - self.curr_att_cos))
        vel_rew = 0.1 * (self.prev_vel - self.curr_vel)
        ang_rew = 0.1 * (self.prev_ang - self.curr_ang)

        ctrl_rew = -1*sum([((a-self.hov_rpm)/self.max_rpm)**2 for a in action])
        ctrl_rew -= 1*sum([((a-pa)/self.max_rpm)**2 for a, pa in zip(action, self.prev_action)])
        #ctrl_rew -= 10*sum([(u-v)**2 for u, v in zip(xyz_dot, self.prev_xyz_dot)])
        #ctrl_rew -= 10*sum([(p-q)**2 for p, q in zip(pqr, self.prev_pqr)])
        
        time_rew = 0.

        total_reward = dist_rew+att_rew+vel_rew+ang_rew+ctrl_rew+time_rew
        return total_reward, {"dist_rew": dist_rew,
                            "att_rew": att_rew,
                            "vel_rew": vel_rew,
                            "ang_rew": ang_rew,
                            "ctrl_rew": ctrl_rew,
                            "time_rew": time_rew}

    def reset(self):
        zeta = self.generate_heading()
        xyz, uvw, pqr = [0, 0, 0], [0, 0, 0], [0, 0, 0]
        state = super(RandomWaypointFHEnv, self).reset_to_custom_state(xyz, zeta, uvw, pqr, self.hov_rpm_)
        self.goal_xyz = self.generate_goal_position()
        xyz, sin_zeta, cos_zeta, uvw, pqr, normalized_rpm = state
        self.set_current_dists((xyz, sin_zeta, cos_zeta, uvw, pqr), self.hov_rpm_)
        obs = self.get_state_obs((xyz, sin_zeta, cos_zeta, uvw, pqr), self.hov_rpm_, normalized_rpm)
        self.set_prev_dists((xyz, sin_zeta, cos_zeta, uvw, pqr), self.hov_rpm_)
        return obs
    
    def generate_heading(self):
        roll, pitch = 0., 0.
        yaw = np.random.uniform(-pi, pi)
        return [roll, pitch, yaw]

    def generate_goal_position(self):
        mag = 10.
        while mag > self.goal_dist:
            xyz = [random.uniform(-self.goal_dist, self.goal_dist) for _ in range(3)]
            mag = np.linalg.norm(xyz)
        return xyz
    
    def render(self, mode='human', video=False, close=False):
        super(RandomWaypointFHEnv, self).render(mode=mode, close=close)
        self.ani.draw_goal(self.goal_xyz)
        self.ani.draw()
        if video: self.ani.save_frame(self.name)
        if close:
            self.ani.close_window()
            self.init_rendering = False