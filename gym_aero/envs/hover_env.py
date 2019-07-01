from gym_aero.envs import env_base
from math import sin, cos
import gym
import numpy as np

class HoverEnv(env_base.AeroEnv):
    def __init__(self):
        super(HoverEnv, self).__init__()
        
        self.goal_xyz = [0, 0, -1.]
        self.goal_zeta = [0, 0, 0]
        self.goal_uvw = [0, 0, 0]
        self.goal_pqr = [0, 0, 0]
        
        self.max_dist = 5
        self.T = 15 

        self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(20,))
    
    def reward(self, xyz, sin_zeta, cos_zeta, uvw, pqr, action):
        # magnitude of the distance from the goal
        # magnitude of the distance from the goal
        curr_dist = sum([(x-g)**2 for x, g in zip(xyz, self.goal_xyz)])**0.5
        curr_att_sin = sum([(s_z - sin(g_s_z))**2 for s_z, g_s_z in zip(sin_zeta, self.goal_zeta)])**0.5
        curr_att_cos = sum([(c_z - cos(g_c_z))**2 for c_z, g_c_z in zip(cos_zeta, self.goal_zeta)])**0.5
        curr_vel = sum([(x-g)**2 for x, g in zip(uvw, self.goal_uvw)])**0.5
        curr_ang = sum([(x-g)**2 for x, g in zip(pqr, self.goal_pqr)])**0.5

        # agent gets a negative reward based on how far away it is from the desired goal state
        dist_rew = 100*(self.prev_dist-curr_dist)
        att_rew = 100*((self.prev_att_sin-curr_att_sin)+(self.prev_att_cos-curr_att_cos))
        vel_rew = 0.1*(self.prev_vel-curr_vel)
        ang_rew = 0.1*(self.prev_ang-curr_ang)
        uvw_accel_rew = -10.*sum([(u-v)**2 for u, v in zip(uvw, self.prev_uvw)])**0.5
        pqr_accel_rew = -10.*sum([(p-q)**2 for p, q in zip(pqr, self.prev_pqr)])**0.5

        # agent gets a negative reward for excessive action inputs
        ctrl_rew = -sum([((a-self.hov_rpm)/self.max_rpm)**2 for a in action])
        ctrl_accel_rew = -sum([((a-pa)/self.max_rpm)**2 for a, pa in zip(action, self.prev_action)])
        
        # agent gets a positive reward for time spent in flight
        time_rew = 10.

        total_reward = dist_rew+att_rew+vel_rew+ang_rew+uvw_accel_rew+pqr_accel_rew+ctrl_rew+ctrl_accel_rew+time_rew

        return total_reward, {"dist_rew": dist_rew, 
                                "att_rew": att_rew,
                                "vel_rew": vel_rew, 
                                "ang_rew": ang_rew, 
                                "uvw_accel_rew": uvw_accel_rew,
                                "pqr_accel_rew": pqr_accel_rew,
                                "ctrl_rew": ctrl_rew,
                                "ctrl_accel_rew": ctrl_accel_rew,
                                "time_rew": time_rew}

    def terminal(self, xyz, zeta, uvw, pqr):
        sq_err = [(x-g)**2 for x, g in zip(xyz, self.goal_xyz)]
        mag = (sum(sq_err))**0.5
        if mag >= self.max_dist:
            #print("Max dist exceeded")
            return True
        elif self.t*self.ctrl_dt >= self.T:
            #print("Time exceeded") 
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
        self.prev_dist = sum([x**2 for x in xyz_obs])**0.5
        self.prev_att_sin = sum([(x-sin(g))**2 for x, g in zip(sin_zeta, self.goal_zeta)])**0.5
        self.prev_att_cos = sum([(x-cos(g))**2 for x, g in zip(cos_zeta, self.goal_zeta)])**0.5
        self.prev_vel = sum([(x-g)**2 for x, g in zip(uvw, self.goal_uvw)])**0.5
        self.prev_ang = sum([(x-g)**2 for x, g in zip(pqr, self.goal_pqr)])**0.5
        self.prev_uvw = uvw
        self.prev_pqr = pqr
        self.prev_action = action
        return next_state
    
    def step(self, action):
        self.t += 1
        action = self.translate_action(action)
        xyz, zeta, uvw, pqr = super(HoverEnv, self).step(action)
        sin_zeta = [sin(z) for z in zeta]
        cos_zeta = [cos(z) for z in zeta]
        curr_rpm = self.get_rpm()
        normalized_rpm = [rpm/self.max_rpm for rpm in curr_rpm]
        reward, info = self.reward(xyz, sin_zeta, cos_zeta, uvw, pqr, action)
        done = self.terminal(xyz, zeta, uvw, pqr)
        obs = self.get_state_obs((xyz, sin_zeta, cos_zeta, uvw, pqr), action, normalized_rpm)
        return obs, reward, done, info

    def reset(self):
        self.t = 0.
        next_state = super(HoverEnv, self).reset()
        xyz, sin_zeta, cos_zeta, uvw, pqr, normalized_rpm = next_state
        obs = self.get_state_obs((xyz, sin_zeta, cos_zeta, uvw, pqr), self.hov_rpm_, normalized_rpm)
        return obs
    
    def render(self, mode='human', close=False):
        super(HoverEnv, self).render(mode=mode, close=close)
        self.ani.draw_goal(self.goal_xyz)
        self.ani.draw()
        if close:
            self.ani.close_window()
            self.init_rendering = False

