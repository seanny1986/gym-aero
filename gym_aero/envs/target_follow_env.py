import numpy as np
import random
from math import pi, sin, cos, acos, tanh, exp
from scipy import interpolate
from gym_aero.envs import path_follow_env
import gym
import numpy as np

class TargetFollowEnv(path_follow_env.PathFollowEnv):
	def __init__(self):
		super(TargetFollowEnv, self).__init__()
		self.name = "TargetFollow-v0"
           
		self.target_dist = 1.
		self.goal_uvw = [0, 0, 0]
		self.goal_pqr = [0, 0, 0]
        
		self.traj_len = 6
		self.max_dist = 5
		self.T = 10
		self._max_episode_steps = int(self.T/self.ctrl_dt)
		self.prev_xyz = [0., 0., 0.]
		
		self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(23,))
    
	def reward(self, xyz, sin_zeta, cos_zeta, uvw, pqr, action):
        # agent gets a negative reward based on how far away it is from the desired goal state
		dist_rew = -10.*(self.curr_dist-self.target_dist)**2
		att_rew = -10.*(self.curr_att_sin+self.curr_att_cos)
		vel_rew = 0.1*(self.prev_vel-self.curr_vel)
		ang_rew = 0.1*(self.prev_ang-self.curr_ang)

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
	
	def get_state_obs(self, state, action, normalized_rpm):
		xyz, sin_zeta, cos_zeta, uvw, pqr = state
		virtual_time = self.t*self.ctrl_dt/self.T
		tar_pos_obs = [x-interpolate.splev(virtual_time, g, der=0) for x, g in zip(xyz, self.spline_objs)]

		target_direction = tar_pos_obs[:-1]+[0.]
		target_direction_mag = sum(k**2 for k in target_direction)**0.5
		target_yaw = acos(sum([x*y for x, y in zip([1., 0., 0.], target_direction)])/target_direction_mag) if target_direction_mag > 0. else 0.
		target_angle = [0., 0.]+[target_yaw]

		zeta_obs = [s_z - sin(ta) for s_z, ta in zip(sin_zeta, target_angle)]+[c_z - cos(ta) for c_z, ta in zip(cos_zeta, target_angle)]
		est_vel = [x2-x1 for x2, x1 in zip(xyz, self.prev_xyz)]
		target_vel_obs = [x-interpolate.splev(virtual_time, g, der=1) for x, g in zip(est_vel, self.spline_objs)]
		vel_obs = [u - g for u, g in zip(uvw, self.goal_uvw)]+[p - g for p, g in zip(pqr, self.goal_pqr)]
		next_state = tar_pos_obs+target_vel_obs+zeta_obs+vel_obs+normalized_rpm+[self.t*self.ctrl_dt]
		return next_state
	
	def set_current_dists(self, state, action, normalized_rpm):
		xyz, sin_zeta, cos_zeta, uvw, pqr = state
		virtual_time = self.t*self.ctrl_dt/self.T
		curr_dist_vec = [x-interpolate.splev(virtual_time, g, der=0) for x, g in zip(xyz, self.spline_objs)]
		target_direction = curr_dist_vec[:-1]+[0.]
		target_direction_mag = sum(k**2 for k in target_direction)**0.5
		target_yaw = acos(sum([x*y for x, y in zip([1., 0., 0.], target_direction)])/target_direction_mag)
		target_angle = [0., 0.]+[target_yaw]
		curr_att_sin_vec = [s_z - sin(ta) for s_z, ta in zip(sin_zeta, target_angle)]
		curr_att_cos_vec = [c_z - cos(ta) for c_z, ta in zip(cos_zeta, target_angle)]
		self.curr_dist = sum([x**2 for x in curr_dist_vec])**0.5
		self.curr_att_sin = sum([sz**2 for sz in curr_att_sin_vec])**0.5
		self.curr_att_cos = self.curr_att_sin = sum([cz**2 for cz in curr_att_cos_vec])**0.5
		self.curr_vel = sum([(x-g)**2 for x, g in zip(uvw, self.goal_uvw)])**0.5
		self.curr_ang = sum([(x-g)**2 for x, g in zip(pqr, self.goal_pqr)])**0.5

	def reset(self):
		state = super(path_follow_env.PathFollowEnv, self).reset()
		self.prev_xyz = [0., 0., 0.]
		self.goal_list_xyz = []
		xyz_ = [1., 0., 0.]
		for _ in range(self.traj_len):
			goal = self.generate_waypoint()
			temp = [x + g for x,g in zip(xyz_, goal)]
			self.goal_list_xyz.append(temp)
			xyz_ = temp
		self.generate_spline()
		xyz, sin_zeta, cos_zeta, uvw, pqr, normalized_rpm = state
		self.set_current_dists((xyz, sin_zeta, cos_zeta, uvw, pqr), self.hov_rpm_, normalized_rpm)
		obs = self.get_state_obs((xyz, sin_zeta, cos_zeta, uvw, pqr), self.hov_rpm_, normalized_rpm)
		self.set_prev_dists((xyz, sin_zeta, cos_zeta, uvw, pqr), self.hov_rpm_, normalized_rpm)
		return obs

	def generate_spline(self):
		xs = np.array([1.]+[wp[0] for wp in self.goal_list_xyz])
		ys = np.array([0.]+[wp[1] for wp in self.goal_list_xyz])
		zs = np.array([0.]+[wp[2] for wp in self.goal_list_xyz])
		t_data = np.linspace(0., 1., self.traj_len+1, endpoint=True)
		x_interp = interpolate.splrep(t_data, xs, s=0)
		y_interp = interpolate.splrep(t_data, ys, s=0)
		z_interp = interpolate.splrep(t_data, zs, s=0)
		self.spline_objs = [x_interp, y_interp, z_interp]
    
	def render(self, mode='human', video=False, close=False):
		super(path_follow_env.PathFollowEnv, self).render(mode=mode, close=close)
		virtual_time = self.t*self.ctrl_dt/self.T
		tar_xyz = [interpolate.splev(virtual_time, g, der=0) for g in self.spline_objs]
		self.ani.draw_goal(tar_xyz, color=(1, 0, 0))
		self.ani.draw()
		if video: self.ani.save_frame("TargetFollow")
		if close:
			self.ani.close_window()
			self.init_rendering = False