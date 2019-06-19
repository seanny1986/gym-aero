import numpy as np
import random
from math import pi, sin, cos, acos, tanh, exp
from scipy import interpolate
from gym_aero.envs import env_base
import gym
import numpy as np

class PathFollowEnv(env_base.AeroEnv):
    def __init__(self):
        super(PathFollowEnv, self).__init__()
        
        self.goal_uvw = [0., 0., 0.]
        self.goal_pqr = [0., 0., 0.]
        self.goal_uvw_next = [0, 0, 0]
        self.goal_pqr_next = [0, 0, 0]

        self.datum = [0., 0., 0.]
        
        self.goal_rad = 1.5
        self.traj_len = 6
        self.goal_thresh = 0.1
        self.max_dist = 5
        self.T = 3.5

        self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(23,))
    
    def reward(self, xyz, sin_zeta, cos_zeta, uvw, pqr, action):
        # evaluate spline functions
        virtual_time = (self.goal*self.T+self.t*self.ctrl_dt)/(self.traj_len*self.T)  
        curr_dist_vec = [x-interpolate.splev(virtual_time, g, der=0) for x, g in zip(xyz, self.spline_objs)]
        target_direction = [xyz[i]-interpolate.splev(virtual_time, self.spline_objs[i], der=1) for i in range(2)]+[0.]
        target_direction_mag = sum(k**2 for k in target_direction)**0.5
        target_yaw = acos(sum([x*y for x, y in zip([1., 0., 0.], target_direction)])/target_direction_mag)
        target_angle = [0., 0.]+[target_yaw]
        curr_att_sin_vec = [s_z - sin(ta) for s_z, ta in zip(sin_zeta, target_angle)]
        curr_att_cos_vec = [c_z - cos(ta) for c_z, ta in zip(cos_zeta, target_angle)]
        
        curr_vel_vec = [x-g for x, g in zip(uvw, self.goal_uvw)]
        curr_ang_vec = [x-g for x, g in zip(pqr, self.goal_pqr)]

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

        self.prev_vel = curr_vel
        self.prev_ang = curr_ang
        self.curr_vel_vec = curr_vel_vec
        self.curr_ang_vec = curr_ang_vec

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
        xyz, _, _, uvw, pqr, normalized_rpm = state
        xyz_obs = [x - g for x, g in zip(xyz, self.goal_xyz)]
        vel_obs = [u - g for u, g in zip(uvw, self.goal_uvw)]+[p - g for p, g in zip(pqr, self.goal_pqr)]
        xyz_next_obs = [x - g for x, g in zip(xyz, self.goal_xyz_next)]
        vel_next_obs = [u - g for u, g in zip(uvw, self.goal_uvw_next)]+[p - g for p, g in zip(pqr, self.goal_pqr_next)]
        curr_tar_obs = xyz_obs+vel_obs
        next_tar_obs = xyz_next_obs+vel_next_obs
        next_state = curr_tar_obs+next_tar_obs+normalized_rpm+[self.t*self.ctrl_dt]
        self.prev_dist = sum([x**2 for x in xyz_obs])**0.5
        self.prev_vel = sum([(x-g)**2 for x, g in zip(uvw, self.goal_uvw)])**0.5
        self.prev_ang = sum([(x-g)**2 for x, g in zip(pqr, self.goal_pqr)])**0.5
        self.prev_action = normalized_rpm
        return next_state

    def step(self, action):
        self.t += 1
        action = self.translate_action(action)
        xyz, zeta, uvw, pqr = super(PathFollowEnv, self).step(action)
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
        next_state = super(PathFollowEnv, self).reset()
        self.goal_list_xyz = []
        xyz = [0., 0., 0.]
        for _ in range(self.traj_len):
            goal = self.generate_waypoint()
            temp = [x + g for x,g in zip(xyz, goal)]
            self.goal_list_xyz.append(temp)
            xyz = temp
        self.generate_spline()
        self.goal = 0
        self.goal_next = self.goal+1
        self.goal_xyz = self.goal_list_xyz[self.goal]
        self.goal_xyz_next = self.goal_list_xyz[self.goal_next]
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
    
    def generate_spline(self):
        xs = np.array([0.]+[wp[0] for wp in self.goal_list_xyz])
        ys = np.array([0.]+[wp[1] for wp in self.goal_list_xyz])
        zs = np.array([0.]+[wp[2] for wp in self.goal_list_xyz])
        t_data = np.linspace(0., 1., self.traj_len+1, endpoint=True)
        x_interp = interpolate.splrep(t_data, xs, s=0)
        y_interp = interpolate.splrep(t_data, ys, s=0)
        z_interp = interpolate.splrep(t_data, zs, s=0)
        self.spline_objs = [x_interp, y_interp, z_interp]
    
    def next_goal(self):
        if self.prev_dist < self.goal_thresh:
            if not self.goal >= len(self.goal_list_xyz)-1:
                self.time_state = float(self.T)
                self.t = 0
                self.goal += 1
                self.goal_xyz = self.goal_list_xyz[self.goal]
            if self.goal_next >= len(self.goal_list_xyz)-1:
                self.goal_xyz_next = [0., 0., 0.]
            else:
                self.goal_next += 1
                self.goal_xyz_next = self.goal_list_xyz[self.goal_next]
    
    def render(self):
        super(PathFollowEnv, self).render(mode='human', close=False)
        steps = 50
        ds = 1./steps
        vals = [i*ds for i in range(steps+1)]
        xyz = [[interpolate.splev(v, o, der=0) for o in self.spline_objs] for v in vals]
        for x in xyz: self.ani.draw_goal(x, color=(1, 0, 0))
        for g in self.goal_list_xyz: self.ani.draw_goal(g)
        for i, g in enumerate([[0., 0., 0.]]+self.goal_list_xyz): 
            if i <= len(self.goal_list_xyz)-1:
                self.ani.draw_line(g, self.goal_list_xyz[i])
        self.ani.draw()