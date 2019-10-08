import gym
import numpy as np
from math import pi, sin, cos, acos, tanh, exp, sqrt
from scipy import interpolate
from gym_aero.envs import env_base

class TrajectoryEnv(env_base.AeroEnv):
    def __init__(self):
        super(TrajectoryEnv, self).__init__()
        
        self.goal_rad = 1.5
        self.traj_len = 4
        self.goal_thresh = 0.1
        self.max_dist = 5
        self.T = 3.5

        self.epsilon_time = 1.5
        self.num_fut_wp = 2
        num_states = 5+15*(self.num_fut_wp+1)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(num_states,))
    
    def switch_goal(self, state):
        if self.curr_dist < self.goal_thresh: return True
        else: return False

    def reward(self, state, action, normalized_rpm):
        xyz, sin_zeta, cos_zeta, uvw, pqr = state

        # agent gets a negative reward based on how far away it is from the desired goal state
        dist_rew = 1000*(self.prev_dist-self.curr_dist)
        att_rew = 0*(self.prev_att_sin+self.prev_att_cos-self.curr_att_sin-self.curr_att_cos)
        vel_rew = 1*(self.prev_vel-self.curr_vel)
        ang_rew = 1*(self.prev_ang-self.curr_ang)

        # agent gets a negative reward for excessive action inputs
        ctrl_rew = -1*sum([((a-self.hov_rpm)/self.max_rpm)**2 for a in action])
        ctrl_dev_rew = -1*sum([((a-pa)/self.max_rpm)**2 for a, pa in zip(action, self.prev_action)])
        dist_dev_rew = -1*sum([(x-y)**2 for x, y in zip(xyz, self.prev_xyz)])
        att_dev_rew = -0*sum([(z-sin(k))**2 for z, k in zip(sin_zeta, self.prev_zeta)])
        att_dev_rew -= 0*sum([(z-cos(k))**2 for z, k in zip(cos_zeta, self.prev_zeta)])
        uvw_dev_rew = -1*sum([(u-v)**2 for u, v in zip(uvw, self.prev_uvw)])
        pqr_dev_rew = -1*sum([(p-q)**2 for p, q in zip(pqr, self.prev_pqr)])
        heading_rew = 0#-1e-2*(uvw[1])**2

        state_rew = dist_rew+att_rew+vel_rew+ang_rew
        dev_rew = ctrl_dev_rew+dist_dev_rew+att_dev_rew+uvw_dev_rew+pqr_dev_rew

        # time reward for staying in the air
        time_rew = 1

        # calculate total reward
        total_reward = state_rew+dev_rew+ctrl_rew+time_rew+heading_rew
        return total_reward, {"dist_rew": dist_rew,
                                "att_rew": att_rew,
                                "vel_rew": vel_rew,
                                "ang_rew": ang_rew,
                                "ctrl_rew": ctrl_rew,
                                "ctrl_dev_rew": ctrl_dev_rew,
                                "dist_dev_rew": dist_dev_rew,
                                "att_dev_rew": att_dev_rew,
                                "uvw_dev_rew": uvw_dev_rew,
                                "pqr_dev_rew": pqr_dev_rew,
                                "heading_rew": heading_rew,
                                "time_rew": time_rew}

    def terminal(self):
        if self.curr_dist >= self.max_dist:
            return True
        elif self.t*self.ctrl_dt >= self.T: 
            return True
        else: 
            return False
    
    def get_obs(self, state, action, normalized_rpm):
        xyz, sin_zeta, cos_zeta, uvw, pqr = state
        xyz_obs = []
        sin_zeta_obs = []
        cos_zeta_obs = []
        uvw_obs = []
        pqr_obs = []
        for i in range(self.num_fut_wp+1):
            if self.goal_counter+i <= len(self.goal_list_xyz)-1:
                xyz_obs = xyz_obs+[x-g for x,g in zip(xyz, self.goal_list_xyz[self.goal_counter+i])]
                sin_zeta_obs = sin_zeta_obs+[z-sin(g) for z,g in zip(sin_zeta, self.goal_list_zeta[self.goal_counter+i])]
                cos_zeta_obs = cos_zeta_obs+[z-cos(g) for z,g in zip(cos_zeta, self.goal_list_zeta[self.goal_counter+i])]
                uvw_obs = uvw_obs+[u-g for u,g in zip(uvw, self.goal_list_uvw[self.goal_counter+i])]
                pqr_obs = pqr_obs+[p-g for p,g in zip(pqr, self.goal_list_pqr[self.goal_counter+i])]
            else:
                xyz_obs = xyz_obs+[0., 0., 0.]
                sin_zeta_obs = sin_zeta_obs+[0., 0., 0.]
                cos_zeta_obs = cos_zeta_obs+[0., 0., 0.]
                uvw_obs = uvw_obs+[0., 0., 0.]
                pqr_obs = pqr_obs+[0., 0., 0.]
        tar_obs = xyz_obs+sin_zeta_obs+cos_zeta_obs+uvw_obs+pqr_obs
        next_state = tar_obs+normalized_rpm+[self.t*self.ctrl_dt]
        return next_state
    
    def set_curr_dists(self, state, action, normalized_rpm):
        xyz, sin_zeta, cos_zeta, uvw, pqr = state
        if not self.goal_counter > self.traj_len-1:
            self.curr_dist = sum([(x-g)**2 for x, g in zip(xyz, self.goal_list_xyz[self.goal_counter])])**0.5
            self.curr_att_sin = sum([(sz-sin(g))**2 for sz, g in zip(sin_zeta, self.goal_list_zeta[self.goal_counter])])**0.5
            self.curr_att_cos = sum([(cz-cos(g))**2 for cz, g in zip(cos_zeta, self.goal_list_zeta[self.goal_counter])])**0.5
            self.curr_vel = sum([(x-g)**2 for x, g in zip(uvw, self.goal_list_uvw[self.goal_counter])])**0.5
            self.curr_ang = sum([(x-g)**2 for x, g in zip(pqr, self.goal_list_pqr[self.goal_counter])])**0.5
    
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
        xyz, zeta, uvw, pqr = super(TrajectoryEnv, self).step(commanded_rpm)
        sin_zeta = [sin(z) for z in zeta]
        cos_zeta = [cos(z) for z in zeta]
        current_rpm = self.get_rpm()
        normalized_rpm = [rpm/self.max_rpm for rpm in current_rpm]
        self.set_current_dists((xyz, sin_zeta, cos_zeta, uvw, pqr), commanded_rpm, normalized_rpm)
        reward, info = self.reward((xyz, sin_zeta, cos_zeta, uvw, pqr), commanded_rpm, normalized_rpm)
        term = self.switch_goal((xyz, sin_zeta, cos_zeta, uvw, pqr))
        if term:
            if self.goal_counter < self.traj_len-1:
                self.goal_counter += 1
                self.set_current_dists((xyz, sin_zeta, cos_zeta, uvw, pqr), commanded_rpm, normalized_rpm)
                self.t = 0
            else: self.t += 1
        else: self.t += 1
        done = self.terminal()
        obs = self.get_obs((xyz, sin_zeta, cos_zeta, uvw, pqr), commanded_rpm, normalized_rpm)
        self.set_prev_dists((xyz, sin_zeta, cos_zeta, uvw, pqr), commanded_rpm, normalized_rpm)
        return obs, reward, done, info

    def reset(self):
        self.t = 0
        self.goal_counter = 0
        self.goal_list_xyz = []
        xyz_ = [0., 0., 0.]
        for _ in range(self.traj_len):
            temp = self.generate_waypoint()
            xyz_ = [x+g for x, g in zip(xyz_, temp)]
            self.goal_list_xyz.append(xyz_)
        self.goal_list_zeta = []
        self.goal_list_uvw = []
        self.goal_list_pqr = []
        for _ in range(self.traj_len):
            self.goal_list_zeta.append([0., 0., 0.])
            self.goal_list_uvw.append([0., 0., 0.])
            self.goal_list_pqr.append([0., 0., 0.])
        state = super(TrajectoryEnv, self).reset()
        xyz, sin_zeta, cos_zeta, uvw, pqr, normalized_rpm = state
        self.set_curr_dists((xyz, sin_zeta, cos_zeta, uvw, pqr), self.hov_rpm_, normalized_rpm)
        self.set_prev_dists((xyz, sin_zeta, cos_zeta, uvw, pqr), self.hov_rpm_, normalized_rpm)
        self.obs = self.get_obs((xyz, sin_zeta, cos_zeta, uvw, pqr), self.hov_rpm_, normalized_rpm)
        return self.obs
    
    def generate_waypoint(self):
        phi = np.random.RandomState().uniform(low=-pi/2, high=pi/2)
        theta = np.random.RandomState().uniform(low=-pi/2, high=pi/2)
        rad = np.random.RandomState().uniform(low=1, high=1.5)
        y = rad*sin(theta)*cos(phi)
        x = rad*cos(theta)*cos(phi)
        z = -rad*sin(theta)
        return [x, y, z]
    
    def render(self, mode='human', close=False):
        super(TrajectoryEnv, self).render(mode=mode, close=close)
        self.ani.draw_goal(np.zeros((3,))) 
        for g in self.goal_list_xyz:
            if np.array_equal(g, self.goal_list_xyz[self.goal_counter]):
                self.ani.draw_goal(g, color=(1., 0., 0.))
            else: 
                self.ani.draw_goal(g) 
        for i, g in enumerate([np.zeros((3,))]+self.goal_list_xyz): 
            if i <= len(self.goal_list_xyz)-1:
                self.ani.draw_line(g, self.goal_list_xyz[i])
        self.ani.draw()
        if close:
            self.ani.close_window()
            self.init_rendering = False