from gym_aero.envs import env_base
from math import sin, cos, acos
import gym
import numpy as np
import random

class RandomWaypointEnv(env_base.AeroEnv):
    def __init__(self):
        super(RandomWaypointEnv, self).__init__()
        
        self.goal_dist = 2.5
        self.max_dist = 5
        self.T = 3

        self.goal_uvw = [0, 0, 0]
        self.goal_pqr = [0, 0, 0]

        self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(20,))
    
    def reward(self, xyz, sin_zeta, cos_zeta, uvw, pqr, action):
        curr_dist_vec = [x-g for x, g in zip(xyz, self.goal_xyz)]
        curr_att_sin_vec = [s_z - sin(g_s_z) for s_z, g_s_z in zip(sin_zeta, self.goal_zeta)]
        curr_att_cos_vec = [c_z - cos(g_c_z) for c_z, g_c_z in zip(cos_zeta, self.goal_zeta)]
        curr_vel_vec = [x-g for x, g in zip(uvw, self.goal_uvw)]
        curr_ang_vec = [x-g for x, g in zip(pqr, self.goal_pqr)]

        # magnitude of the distance from the goal
        curr_dist = (sum([cd**2 for cd in curr_dist_vec]))**0.5
        curr_att_sin = (sum([cd**2 for cd in curr_att_sin_vec]))**0.5
        curr_att_cos = (sum([cd**2 for cd in curr_att_cos_vec]))**0.5
        curr_vel = (sum([cd**2 for cd in curr_vel_vec]))**0.5
        curr_ang = (sum([cd**2 for cd in curr_ang_vec]))**0.5

        # agent gets a negative reward based on how far away it is from the desired goal state
        dist_rew = 100*(self.prev_dist-curr_dist)
        att_rew = 10*((self.prev_att_sin-curr_att_sin)+(self.prev_att_cos-curr_att_cos))
        vel_rew = 0.1*(self.prev_vel-curr_vel)
        ang_rew = 0.1*(self.prev_ang-curr_ang)

        self.prev_dist = curr_dist
        self.prev_att_sin = curr_att_sin
        self.prev_att_cos = curr_att_cos
        self.prev_vel = curr_vel
        self.prev_ang = curr_ang

        self.curr_dist_vec = curr_dist_vec
        self.curr_att_sin_vec = curr_att_sin_vec
        self.curr_att_cos_vec = curr_att_cos_vec
        self.curr_vel_vec = curr_vel_vec
        self.curr_ang_vec = curr_ang_vec

        # agent gets a negative reward for excessive action inputs
        ctrl_rew = 0.
        ctrl_rew -= sum([((a-self.hov_rpm)/self.max_rpm)**2 for a in action])
        ctrl_rew -= sum([((a-pa)/self.max_rpm)**2 for a, pa in zip(action, self.prev_action)])
        ctrl_rew -= 10.*sum([(u-v)**2 for u, v in zip(uvw, self.prev_uvw)])
        ctrl_rew -= 10.*sum([(p-q)**2 for p, q in zip(pqr, self.prev_pqr)])

        # agent gets a positive reward for time spent in flight
        time_rew = 10.

        total_reward = dist_rew+att_rew+vel_rew+ang_rew+ctrl_rew+time_rew
        return total_reward, {"dist_rew": dist_rew, 
                                "att_rew": att_rew, 
                                "vel_rew": vel_rew, 
                                "ang_rew": ang_rew, 
                                "ctrl_rew": ctrl_rew, 
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
    
    def get_state_obs(self, state):
        xyz, sin_zeta, cos_zeta, uvw, pqr, normalized_rpm = state
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
        self.prev_action = normalized_rpm
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
        obs = self.get_state_obs((xyz, sin_zeta, cos_zeta, uvw, pqr, normalized_rpm))
        return obs, reward, done, info

    def reset(self):
        next_state = super(RandomWaypointEnv, self).reset()
        self.goal_xyz = self.generate_goal_position()
        self.goal_zeta = self.generate_goal_zeta()
        obs = self.get_state_obs(next_state)
        return obs
    
    def generate_goal_position(self):
        mag = 10.
        while mag > self.goal_dist:
            x = random.uniform(0., self.goal_dist)
            y = random.uniform(-self.goal_dist, self.goal_dist)
            z = random.uniform(-self.goal_dist, self.goal_dist)
            mag = (x**2+y**2+z**2)**0.5
        print([x, y, z])
        return [x, y, z]

    def generate_goal_zeta(self):
        vec = [g-p for g, p in zip(self.goal_xyz[:-1], [0., 0.])]+[0.]
        x_dir = [1., 0., 0.]
        dot_product = sum([v*x for v, x in zip(vec, x_dir)])
        mag_vec = sum([v**2 for v in vec])**0.5
        roll, pitch = 0., 0.
        yaw = acos(dot_product/mag_vec)
        return [roll, pitch, yaw]
    
    def render(self, mode='human', close=False):
        super(RandomWaypointEnv, self).render(mode='human', close=False)
        self.ani.draw_goal(self.goal_xyz)
        self.ani.draw()