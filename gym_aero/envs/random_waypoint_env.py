from gym_aero.envs import env_base
from math import sin, cos, acos, pi
import gym
import numpy as np
import random

class RandomWaypointEnv(env_base.AeroEnv):
    def __init__(self):
        super(RandomWaypointEnv, self).__init__()
        
        self.goal_dist = 1.5
        self.max_dist = 5
        self.T = 5

        self.goal_uvw = [0., 0., 0.]
        self.goal_pqr = [0., 0., 0.]
        self.goal_zeta = [0., 0., 0.]

        self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(20,))
    
    def reward(self, xyz, sin_zeta, cos_zeta, uvw, pqr, action):
        # magnitude of the distance from the goal
        curr_dist = np.linalg.norm([x-g for x, g in zip(xyz, self.goal_xyz)])
        curr_att_sin = np.linalg.norm([s_z-sin(g) for s_z, g in zip(sin_zeta, self.goal_zeta)])
        curr_att_cos = np.linalg.norm([c_z-cos(g) for c_z, g in zip(cos_zeta, self.goal_zeta)])
        curr_vel = np.linalg.norm([u-g for u, g in zip(uvw, self.goal_uvw)])
        curr_ang = np.linalg.norm([p-g for p, g in zip(pqr, self.goal_pqr)])

        # agent gets a negative reward based on how far away it is from the desired goal state
        dist_rew = 100.*(self.prev_dist-curr_dist)
        #print(self.prev_att_sin-curr_att_sin, self.prev_att_cos-curr_att_cos)
        att_rew = 100.*((self.prev_att_sin-curr_att_sin)+(self.prev_att_cos-curr_att_cos))
        vel_rew = 50.*(self.prev_vel-curr_vel)
        ang_rew = 50.*(self.prev_ang-curr_ang)
        uvw_accel_rew = -50.*sum([(u-v)**2 for u, v in zip(uvw, self.prev_uvw)])**0.5
        pqr_accel_rew = -50.*sum([(p-q)**2 for p, q in zip(pqr, self.prev_pqr)])**0.5

        # agent gets a negative reward for excessive action inputs
        ctrl_rew = -1.*sum([((a-self.hov_rpm)/self.max_rpm)**2 for a in action])
        ctrl_accel_rew = -1.*sum([((a-pa)/self.max_rpm)**2 for a, pa in zip(action, self.prev_action)])
        
        # agent gets a positive reward for time spent in flight
        time_rew = 0.

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
        sin_zeta_obs = [sz - sin(g) for sz, g in zip(sin_zeta, self.goal_zeta)]
        cos_zeta_obs = [cz - cos(g) for cz, g in zip(cos_zeta, self.goal_zeta)]
        uvw_obs = [u - g for u, g in zip(uvw, self.goal_uvw)]
        pqr_obs = [p - g for p, g in zip(pqr, self.goal_pqr)]
        curr_tar_obs = xyz_obs+sin_zeta_obs+cos_zeta_obs+uvw_obs+pqr_obs
        next_state = curr_tar_obs+normalized_rpm+[self.t*self.ctrl_dt]
        self.prev_dist = np.linalg.norm([x-g for x, g in zip(xyz, self.goal_xyz)])
        self.prev_att_sin = np.linalg.norm([s_z-sin(g) for s_z, g in zip(sin_zeta, self.goal_zeta)])
        self.prev_att_cos = np.linalg.norm([c_z-cos(g) for c_z, g in zip(cos_zeta, self.goal_zeta)])
        self.prev_vel = np.linalg.norm([u-g for u, g in zip(uvw, self.goal_uvw)])
        self.prev_ang = np.linalg.norm([p-g for p, g in zip(pqr, self.goal_pqr)])
        self.prev_uvw = uvw
        self.prev_pqr = pqr
        self.prev_action = action
        return next_state
    
    def step(self, action):
        self.t += 1
        action = self.translate_action(action)
        xyz, zeta, uvw, pqr = super(RandomWaypointEnv, self).step(action)
        sin_zeta = [sin(z) for z in zeta]
        cos_zeta = [cos(z) for z in zeta]
        curr_rpm = self.get_rpm()
        normalized_rpm = [rpm/self.max_rpm for rpm in curr_rpm]
        reward, info = self.reward(xyz, sin_zeta, cos_zeta, uvw, pqr, action)
        done = self.terminal(xyz, zeta, uvw, pqr)
        obs = self.get_state_obs((xyz, sin_zeta, cos_zeta, uvw, pqr), action, normalized_rpm)
        return obs, reward, done, info

    def reset(self):
        next_state = super(RandomWaypointEnv, self).reset()
        self.goal_xyz = self.generate_goal_position()
        xyz, sin_zeta, cos_zeta, uvw, pqr, normalized_rpm = next_state
        obs = self.get_state_obs((xyz, sin_zeta, cos_zeta, uvw, pqr), self.hov_rpm_, normalized_rpm)
        return obs
    
    def generate_goal_position(self):
        mag = 10.
        while mag > self.goal_dist:
            xyz = [random.uniform(-self.goal_dist, self.goal_dist) for _ in range(3)]
            mag = np.linalg.norm(xyz)
        return xyz

    def generate_goal_zeta(self):
        vec = [g-p for g, p in zip(self.goal_xyz[:-1], [0., 0.])]+[0.]
        x_dir = [1., 0., 0.]
        dot_product = sum([v*x for v, x in zip(vec, x_dir)])
        mag_vec = sum([v**2 for v in vec])**0.5
        roll, pitch = 0., 0.
        yaw = acos(dot_product/mag_vec)
        return [roll, pitch, yaw]
    
    def render(self, mode='human', close=False):
        super(RandomWaypointEnv, self).render(mode=mode, close=close)
        self.ani.draw_goal(self.goal_xyz)
        self.ani.draw()
        if close:
            self.ani.close_window()
            self.init_rendering = False