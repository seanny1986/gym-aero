import gym
import random
import numpy as np
from math import pi, sin, cos, acos, tanh, exp
from scipy import interpolate
from gym_aero.envs import env_base

class TrajectoryEnv(env_base.AeroEnv):
    def __init__(self):
        super(TrajectoryEnv, self).__init__()
        
        self.goal_uvw = [0, 0, 0]
        self.goal_pqr = [0, 0, 0]
        self.goal_uvw_next = [0, 0, 0]
        self.goal_pqr_next = [0, 0, 0]
        
        self.goal_rad = 1.5
        self.traj_len = 6
        self.goal_thresh = 0.1
        self.max_dist = 5
        self.T = 3.5

        self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(35,))
    
    def reward(self, xyz, sin_zeta, cos_zeta, uvw, pqr, action):
        curr_dist_vec = [x - g for x, g in zip(xyz, self.goal_xyz)]
        curr_att_sin_vec = [sz - sin(g) for sz, g in zip(sin_zeta, self.goal_zeta)]
        curr_att_cos_vec = [cz - cos(g) for cz, g in zip(cos_zeta, self.goal_zeta)]
        curr_vel_vec = [x - g for x, g in zip(uvw, self.goal_uvw)]
        curr_ang_vec = [x - g for x, g in zip(pqr, self.goal_pqr)]

        # magnitude of the distance from the goal
        curr_dist = (sum([cd**2 for cd in curr_dist_vec]))**0.5
        curr_att_sin = (sum([cd**2 for cd in curr_att_sin_vec]))**0.5
        curr_att_cos = (sum([cd**2 for cd in curr_att_cos_vec]))**0.5
        curr_vel = (sum([cd**2 for cd in curr_vel_vec]))**0.5
        curr_ang = (sum([cd**2 for cd in curr_ang_vec]))**0.5

        # agent gets a negative reward based on how far away it is from the desired goal state
        dist_rew = -10.*curr_dist
        att_rew = -10.*(curr_att_sin+curr_att_cos)
        vel_rew = 0.1*(self.prev_vel-curr_vel)
        ang_rew = 0.1*(self.prev_ang-curr_ang)

        # agent gets a negative reward for excessive action inputs
        ctrl_rew = 0.
        ctrl_rew -= sum([((a-self.hov_rpm)/self.max_rpm)**2 for a in action])
        ctrl_rew -= sum([((a-pa)/self.max_rpm)**2 for a, pa in zip(action, self.prev_action)])
        ctrl_rew -= 10.*sum([(u-v)**2 for u, v in zip(uvw, self.prev_uvw)])
        ctrl_rew -= 10.*sum([(p-q)**2 for p, q in zip(pqr, self.prev_pqr)])

        time_rew = 0.

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
        xyz_next_obs = [x - g for x, g in zip(xyz, self.goal_xyz_next)]
        zeta_next_obs = [sz - sin(g) for sz, g in zip(sin_zeta, self.goal_zeta_next)]+[cz - cos(g) for cz, g in zip(cos_zeta, self.goal_zeta_next)]
        vel_next_obs = [u - g for u, g in zip(uvw, self.goal_uvw_next)]+[p - g for p, g in zip(pqr, self.goal_pqr_next)]
        curr_tar_obs = xyz_obs+zeta_obs+vel_obs
        next_tar_obs = xyz_next_obs+zeta_next_obs+vel_next_obs
        next_state = curr_tar_obs+next_tar_obs+normalized_rpm+[self.t*self.ctrl_dt]
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
        xyz, zeta, uvw, pqr = super(TrajectoryEnv, self).step(action)
        sin_zeta = [sin(z) for z in zeta]
        cos_zeta = [cos(z) for z in zeta]
        curr_rpm = self.get_rpm()
        normalized_rpm = [rpm/self.max_rpm for rpm in curr_rpm]
        reward, info = self.reward(xyz, sin_zeta, cos_zeta, uvw, pqr, action)
        done = self.terminal(xyz, zeta, uvw, pqr)
        self.next_goal()
        obs = self.get_state_obs((xyz, sin_zeta, cos_zeta, uvw, pqr, normalized_rpm))
        return obs, reward, done, info

    def reset(self):
        next_state = super(TrajectoryEnv, self).reset()
        self.goal_list_xyz = []
        xyz = [0., 0., 0.]
        for _ in range(self.traj_len):
            goal = self.generate_waypoint()
            temp = [x + g for x,g in zip(xyz, goal)]
            self.goal_list_xyz.append(temp)
            xyz = temp
        self.goal_list_zeta = []
        i = self.traj_len-2
        while True:
            temp = self.goal_list_xyz[i+1]
            xyz = [0., 0., 0.] if i < 0 else self.goal_list_xyz[i] 
            yaw = self.generate_yaw(temp, xyz)
            self.goal_list_zeta.append(yaw)
            if i < 0: break
            i -= 1
        self.goal = 0
        self.goal_next = self.goal+1
        self.goal_xyz = self.goal_list_xyz[self.goal]
        self.goal_xyz_next = self.goal_list_xyz[self.goal_next]
        self.goal_zeta = self.goal_list_zeta[self.goal]
        self.goal_zeta_next = self.goal_list_zeta[self.goal_next]
        obs = self.get_state_obs(next_state)
        return obs
    
    def generate_waypoint(self):
        mag = 10.
        while mag > self.goal_rad:
            x = random.uniform(0, self.goal_rad)
            y = random.uniform(-self.goal_rad, self.goal_rad)
            z = random.uniform(-self.goal_rad, self.goal_rad)
            mag = (x**2+y**2+z**2)**0.5
        return [x, y, z]
        
    def next_goal(self):
        if self.prev_dist < self.goal_thresh:
            if not self.goal >= len(self.goal_list_xyz)-1:
                self.time_state = float(self.T)
                self.t = 0
                self.goal += 1
                self.goal_xyz = self.goal_list_xyz[self.goal]
                self.goal_zeta = self.goal_list_zeta[self.goal]
            if self.goal_next >= len(self.goal_list_xyz)-1:
                self.goal_xyz_next = [0., 0., 0.]
                self.goal_zeta_next = [0., 0., 0.]
            else:
                self.goal_next += 1
                self.goal_xyz_next = self.goal_list_xyz[self.goal_next]
                self.goal_zeta_next = self.goal_list_zeta[self.goal_next]
    
    def generate_yaw(self, v1, v2):
        direction = [u - v for u, v in zip(v1, v2)]
        xy = direction[:-1]+[0.]
        mag = sum([x**2 for x in xy])**0.5
        target_yaw = acos(sum([x*y for x, y in zip([1., 0., 0.], xy)])/mag)
        target_angle = [0., 0.]+[target_yaw]
        return target_angle
    
    def render(self):
        super(TrajectoryEnv, self).render(mode='human', close=False)
        for g in [[0., 0., 0.]]+self.goal_list_xyz: 
            if g == self.goal_xyz:
                self.ani.draw_goal(g, color=(1., 0., 0.))
            else: 
                self.ani.draw_goal(g) 
        for i, g in enumerate([[0., 0., 0.]]+self.goal_list_xyz): 
            if i <= len(self.goal_list_xyz)-1:
                self.ani.draw_line(g, self.goal_list_xyz[i])
        self.ani.draw()