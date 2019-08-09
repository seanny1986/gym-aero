import gym
import random
import numpy as np
from math import pi, sin, cos, acos, tanh, exp, sqrt
from scipy import interpolate
from gym_aero.envs import env_base

class TrajectoryEnv(env_base.AeroEnv):
    def __init__(self):
        super(TrajectoryEnv, self).__init__()
        
        self.goal_uvw = [0, 0, 0]
        self.goal_pqr = [0, 0, 0]
        self.goal_uvw_next = [0, 0, 0]
        self.goal_pqr_next = [0, 0, 0]
        
        self.goal_rad = 1.
        self.traj_len = 4
        self.goal_thresh = 0.1
        self.max_dist = 5
        self.T = 3.5

        self.epsilon_time = 1.5

        self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(35,))
    
    def reward(self, state, action, normalized_rpm):
        xyz, sin_zeta, cos_zeta, uvw, pqr = state

        # agent gets a negative reward based on how far away it is from the desired goal state
        dist_rew = 100*(self.prev_dist-self.curr_dist)
        att_rew = 100*(self.prev_att_sin+self.prev_att_cos-self.curr_att_sin-self.curr_att_cos)
        vel_rew = 50*(self.prev_vel-self.curr_vel)
        ang_rew = 50*(self.prev_ang-self.curr_ang)

        # agent gets a negative reward for excessive action inputs
        ctrl_rew = -sum([((a-self.hov_rpm)/self.max_rpm)**2 for a in action])
        ctrl_rew -= sum([((a-pa)/self.max_rpm)**2 for a, pa in zip(action, self.prev_action)])
        ctrl_rew -= 10*sum([(x-y)**2 for x, y in zip(xyz, self.prev_xyz)])
        ctrl_rew -= 10*sum([(z-sin(k))**2 for z, k in zip(sin_zeta, self.prev_zeta)])
        ctrl_rew -= 10*sum([(z-cos(k))**2 for z, k in zip(cos_zeta, self.prev_zeta)])
        ctrl_rew -= 10*sum([(u-v)**2 for u, v in zip(uvw, self.prev_uvw)])
        ctrl_rew -= 10*sum([(p-q)**2 for p, q in zip(pqr, self.prev_pqr)])

        # time reward for staying in the air
        time_rew = 0.

        # calculate total reward
        total_reward = dist_rew+att_rew+vel_rew+ang_rew+ctrl_rew+time_rew
        return total_reward, {"dist_rew": dist_rew, 
                                "att_rew": att_rew, 
                                "vel_rew": vel_rew,
                                "ang_rew": ang_rew,
                                "ctrl_rew": ctrl_rew,
                                "time_rew": time_rew}

    def terminal(self):
        if self.goal == self.traj_len and self.curr_dist <= self.goal_thresh:
            self.flagged = True

        if self.curr_dist >= self.max_dist:
            return True
        elif self.t*self.ctrl_dt > self.T: 
            if not self.flagged:
                return True
            else:
                return False
        elif self.flag_counter >= self.epsilon_time:
            return True
        else: 
            return False
    
    def get_state_obs(self, state, action, normalized_rpm):
        xyz, sin_zeta, cos_zeta, uvw, pqr = state
        xyz_obs = [x-g for x,g in zip(xyz, self.goal_xyz)]
        zeta_obs = [z-sin(g) for z,g in zip(sin_zeta, self.goal_zeta)]+[z-cos(g) for z,g in zip(cos_zeta, self.goal_zeta)]
        vel_obs = [u-g for u,g in zip(uvw, self.goal_uvw)]+[p-g for p,g in zip(pqr, self.goal_pqr)]
        xyz_next_obs = [x-g for x,g in zip(xyz, self.goal_xyz_next)]
        zeta_next_obs = [z-sin(g) for z,g in zip(sin_zeta, self.goal_zeta_next)]+[z-cos(g) for z, g in zip(cos_zeta, self.goal_zeta_next)]
        vel_next_obs = [u-g for u,g in zip(uvw, self.goal_uvw_next)]+[p-g for p,g in zip(pqr, self.goal_pqr_next)]
        curr_tar_obs = xyz_obs+zeta_obs+vel_obs
        next_tar_obs = xyz_next_obs+zeta_next_obs+vel_next_obs
        next_state = curr_tar_obs+next_tar_obs+normalized_rpm+[self.t*self.ctrl_dt]
        return next_state
    
    def set_current_dists(self, state, action, normalized_rpm):
        xyz, sin_zeta, cos_zeta, uvw, pqr = state
        self.curr_dist = sum([(x-g)**2 for x, g in zip(xyz, self.goal_xyz)])**0.5
        self.curr_att_sin = sum([(sz-sin(g))**2 for sz, g in zip(sin_zeta, self.goal_zeta)])**0.5
        self.curr_att_cos = sum([(cz-cos(g))**2 for cz, g in zip(cos_zeta, self.goal_zeta)])**0.5
        self.curr_vel = sum([(x-g)**2 for x, g in zip(uvw, self.goal_uvw)])**0.5
        self.curr_ang = sum([(x-g)**2 for x, g in zip(pqr, self.goal_pqr)])**0.5
    
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
        if self.curr_dist <= self.goal_thresh: self.next_goal()
        done = self.terminal()
        obs = self.get_state_obs((xyz, sin_zeta, cos_zeta, uvw, pqr), commanded_rpm, normalized_rpm)
        self.set_prev_dists((xyz, sin_zeta, cos_zeta, uvw, pqr), commanded_rpm, normalized_rpm)
        self.t += 1
        if self.flagged: self.flag_counter += self.ctrl_dt
        return obs, reward, done, info

    def reset(self):
        self.flag_counter = 0
        self.flagged = False
        self.traj_len = np.random.randint(low=1, high=10)

        # terminate previous sim, initialize new one
        state = super(TrajectoryEnv, self).reset()
        xyz, sin_zeta, cos_zeta, uvw, pqr, normalized_rpm = state
        
        # generate waypoint positions
        self.goal_list_xyz = []
        xyz_temp = [0., 0., 0.]
        for _ in range(self.traj_len):
            goal = self.generate_waypoint()
            temp = [x + g for x,g in zip(xyz_temp, goal)]
            self.goal_list_xyz.append(temp)
            xyz_temp = temp
        
        # generate goal angles
        self.goal_list_zeta = []
        i = self.traj_len-2
        prev_yaw = None
        running = True
        while running:
            temp = self.goal_list_xyz[i+1]
            xyz = [0., 0., 0.] if i < 0 else self.goal_list_xyz[i]
            yaw = self.generate_yaw(temp, xyz, prev_yaw)
            self.goal_list_zeta.append(yaw)
            prev_yaw = yaw[-1]
            if i < 0: running = False
            i -= 1
        
        # set current goal, next goal
        self.goal = 0
        self.goal_next = self.goal+1
        self.goal_xyz = self.goal_list_xyz[self.goal]
        self.goal_zeta = self.goal_list_zeta[self.goal]
        
        if self.goal_next >= len(self.goal_list_xyz)-1:
            self.goal_xyz_next = [0., 0., 0.]
            self.goal_zeta_next = [0., 0., 0.]
        else:
            self.goal_xyz_next = self.goal_list_xyz[self.goal_next]
            self.goal_zeta_next = self.goal_list_zeta[self.goal_next]
        
        # calculate current distance to goals
        self.set_current_dists((xyz, sin_zeta, cos_zeta, uvw, pqr), self.hov_rpm_, normalized_rpm)
        
        # get state observation
        obs = self.get_state_obs((xyz, sin_zeta, cos_zeta, uvw, pqr), self.hov_rpm_, normalized_rpm)
        
        # set previous distances
        self.set_prev_dists((xyz, sin_zeta, cos_zeta, uvw, pqr), self.hov_rpm_, normalized_rpm)
        return obs
    
    def generate_waypoint(self):
        mag = 10.
        while mag > self.goal_rad:
            x = random.uniform(0, self.goal_rad)
            y = random.uniform(-self.goal_rad, self.goal_rad)
            z = random.uniform(-self.goal_rad, self.goal_rad)
            mag = sqrt(x**2+y**2+z**2)
        return [x, y, z]
        
    def next_goal(self):
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
    
    def generate_yaw(self, v1, v2, prev_yaw):
        direction = [u-v for u, v in zip(v1, v2)]
        xy = direction[:-1]+[0.]
        mag = sqrt(sum([x**2 for x in xy]))
        target_yaw = acos(sum([x*y for x, y in zip([1., 0., 0.], xy)])/mag)
        if prev_yaw is not None: target_yaw = (target_yaw+prev_yaw)/2.
        target_angle = [0., 0.]+[target_yaw]
        return target_angle
    
    def render(self, mode='human', close=False):
        super(TrajectoryEnv, self).render(mode=mode, close=close)
        for g in [[0., 0., 0.]]+self.goal_list_xyz: 
            if g == self.goal_xyz:
                self.ani.draw_goal(g, color=(1., 0., 0.))
            else: 
                self.ani.draw_goal(g) 
        for i, g in enumerate([[0., 0., 0.]]+self.goal_list_xyz): 
            if i <= len(self.goal_list_xyz)-1:
                self.ani.draw_line(g, self.goal_list_xyz[i])
        self.ani.draw()
        if close:
            self.ani.close_window()
            self.init_rendering = False