import simulation.quadrotor3 as quad
import simulation.config as cfg
import numpy as np
import random
from math import pi, sin, cos, tanh, exp
import gym
from gym import error, spaces, utils
from gym.utils import seeding
import simulation.animation_gl as ani_gl
import time
from scipy import interpolate

"""
    Environment wrapper for an aerobatic loop task. The goal of this task is for the agent to climb from [0, 0, 0]^T
    to [0, 0, 1.5]^T, and to remain at that altitude until the the episode terminates at T=15s.
"""

class LoopEnv(gym.Env):
    def __init__(self):
        metadata = {'render.modes': ['human']}
        
        # environment parameters
        self.goal_xyz = np.array([[0.],
                                [0.],
                                [0.]])
        
        rad = 2.
        start_theta = (3/2)*pi
        thetas = np.linspace(start_theta, start_theta+2*pi, 10)
        xs = np.zeros((10,))
        ys = rad*np.cos(thetas)+rad
        zs = rad*np.sin(thetas)+rad

        phi = np.linspace(0,2*pi, 10)
        theta = np.zeros((10,))
        psi = np.zeros((10,))
        
        self.pos = [np.vstack([x, y, z]) for x, y, z in zip(xs, ys, zs)]
        self.rot = [np.vstack([p, t, s]) for p, t, s in zip(phi, theta, psi)]

        self.x_interp, self.y_interp, self.z_interp = self.generate_spline(self.pos)
        self.phi_interp, self.theta_interp, self.psi_interp = self.generate_spline(self.rot)
        
        self.t_thresh = 0.1
        self.t = 0
        self.T = 3.
        self.action_space = np.zeros((4,))
        self.observation_space = np.zeros((20,))

        # simulation parameters
        self.params = cfg.params
        self.iris = quad.Quadrotor(self.params)
        self.sim_dt = self.params["dt"]
        self.ctrl_dt = 0.05
        self.steps = range(int(self.ctrl_dt/self.sim_dt))
        self.action_bound = [0, self.iris.max_rpm]
        self.H = int(self.T/self.ctrl_dt)
        self.hov_rpm = self.iris.hov_rpm
        self.trim = [self.hov_rpm, self.hov_rpm,self.hov_rpm, self.hov_rpm]
        self.trim_np = np.array(self.trim)
        self.bandwidth = 35.

        self.max_thrust = self.iris.max_thrust
        self.max_rpm = self.iris.max_rpm

        self.init_rendering = False

    def get_goal(self):
        return self.goal_xyz

    def reward(self, state, action):
        xyz, zeta, uvw, pqr = state
        s_zeta = np.sin(zeta)
        c_zeta = np.cos(zeta)

        virtual_time = self.time_mult*self.t*self.ctrl_dt/self.T

        target_position_x = interpolate.splev(virtual_time, self.x_interp, der=0)
        target_position_y = interpolate.splev(virtual_time, self.y_interp, der=0)
        target_position_z = interpolate.splev(virtual_time, self.z_interp, der=0)

        target_rotation_x = interpolate.splev(virtual_time, self.phi_interp, der=0)
        target_rotation_y = interpolate.splev(virtual_time, self.theta_interp, der=0)
        target_rotation_z = interpolate.splev(virtual_time, self.psi_interp, der=0)

        target_pos = np.array([[target_position_x],[target_position_y],[target_position_z]])
        target_rot = np.array([[target_rotation_x],[target_rotation_y],[target_rotation_z]])
        
        target_vec = xyz-target_pos
        sin_vec = s_zeta-np.sin(target_rot)
        cos_vec = c_zeta-np.cos(target_rot)

        target_dist = np.linalg.norm(target_vec)
        rot_dist = (np.linalg.norm(sin_vec)+np.linalg.norm(cos_vec))/2.

        if target_dist < self.t_thresh:
            pos_rew = (2.-tanh(target_dist))
            att_rew = (2.-tanh(rot_dist))
        else:
            pos_rew = 0.
            att_rew = 0.
        
        # agent gets a negative reward for excessive action inputs
        ctrl_rew = 0.
        ctrl_rew -= np.sum(((action-self.trim_np)/self.action_bound[1])**2)
        ctrl_rew -= np.sum((((action-self.prev_action)/self.action_bound[1])**2))
        ctrl_rew -= np.linalg.norm(uvw-self.prev_uvw)
        ctrl_rew -= np.linalg.norm(pqr-self.prev_pqr)
        
        # agent gets a positive reward for time spent in flight
        time_rew = 0.
        
        return pos_rew, att_rew, ctrl_rew, time_rew

    def terminal(self, pos):
        """

        Parameters
        ----------
        pos :

        Returns
        -------
            bool (boolean) : 
                a boolean value determining whether or not the simulation should be
                terminated.
        """
        xyz, zeta = pos
        mask3 = np.linalg.norm(xyz) > 4.
        if np.sum(mask3) > 0:
            return True
        elif self.ctrl_dt*self.t >= self.T-self.ctrl_dt:
            return True
        else:
            return False

    def step(self, action):
        """

        Parameters
        ----------
        action :

        Returns
        -------
        ob, reward, episode_over, info : tuple
            ob (object) :
                an environment-specific object representing your observation of
                the environment.
            reward (float) :
                amount of reward achieved by the previous action. The scale
                varies between environments, but the goal is always to increase
                your total reward.
            episode_over (bool) :
                whether it's time to reset the environment again. Most (but not
                all) tasks are divided up into well-defined episodes, and done
                being True indicates the episode has terminated. (For example,
                perhaps the pole tipped too far, or you lost your last life.)
            info (dict) :
                 diagnostic information useful for debugging. It can sometimes
                 be useful for learning (for example, it might contain the raw
                 probabilities behind the environment's last state change).
                 However, official evaluations of your agent are not allowed to
                 use this for learning.
        """

        a = action*self.max_rpm
        for _ in self.steps:
            xyz, zeta, uvw, pqr = self.iris.step(a)

        #print("Action: {}".format(a/self.iris.max_thrust))
        sin_zeta = np.sin(zeta)
        cos_zeta = np.cos(zeta)
        current_rpm = (self.iris.get_rpm()/self.action_bound[1]).tolist()
        next_position = xyz.T.tolist()[0]
        next_attitude = sin_zeta.T.tolist()[0]+cos_zeta.T.tolist()[0]
        next_velocity = uvw.T.tolist()[0]+pqr.T.tolist()[0]
        next_state = next_position+next_attitude+next_velocity
        info = self.reward((xyz, zeta, uvw, pqr), action)
        done = self.terminal((xyz, zeta))
        reward = sum(info)
        next_state = next_state+current_rpm+[self.t*self.ctrl_dt]
        self.prev_action = action.copy()
        self.prev_uvw = uvw.copy()
        self.prev_pqr = pqr.copy()
        self.t += 1
        self.time_mult = tanh(self.t/50.)
        return next_state, reward, done, {"pos_rew": info[0],
                                        "att_rew": info[1],
                                        "ctrl_rew": info[2],
                                        "time_rew": info[3]}

    def generate_spline(self, goal_list):
        xs = np.array([0.]+[wp[0,0] for wp in goal_list])
        ys = np.array([0.]+[wp[1,0] for wp in goal_list])
        zs = np.array([0.]+[wp[2,0] for wp in goal_list])
        t_data = np.linspace(0., 1., len(goal_list)+1, endpoint=True)
        x_interp = interpolate.splrep(t_data, xs, s=0)
        y_interp = interpolate.splrep(t_data, ys, s=0)
        z_interp = interpolate.splrep(t_data, zs, s=0)
        return x_interp, y_interp, z_interp

    def reset(self):
        self.t = 0
        self.time_mult = tanh(self.t)
        xyz, zeta, uvw, pqr = self.iris.reset()
        self.iris.set_rpm(np.array(self.trim))
        sin_zeta = np.sin(zeta)
        cos_zeta = np.cos(zeta)
        current_rpm = (self.iris.get_rpm()/self.action_bound[1]).tolist()
        next_position = xyz.T.tolist()[0]
        next_attitude = sin_zeta.T.tolist()[0]+cos_zeta.T.tolist()[0]
        next_velocity = uvw.T.tolist()[0]+pqr.T.tolist()[0]  
        next_state = next_position+next_attitude+next_velocity
        next_state = next_state+current_rpm+[self.t*self.ctrl_dt]
        self.prev_action = self.trim_np.copy()
        self.prev_uvw = np.array([[0.],[0.],[0.]])
        self.prev_pqr = np.array([[0.],[0.],[0.]])
        return next_state
    
    def render(self, mode='human', close=False):
        if not self.init_rendering:
            self.ani = ani_gl.VisualizationGL(name="Static Waypoint")
            self.init_rendering = True
        
        """
        for i in range(25):
            t = i/24.
            x = interpolate.splev(t, self.x_interp, der=0)
            y = interpolate.splev(t, self.y_interp, der=0)
            z = interpolate.splev(t, self.z_interp, der=0)
            p = np.array([[x],[y],[z]])
            self.ani.draw_goal(p, color=(0.1,0.,0.1))
        """
        virtual_time = self.time_mult*self.t*self.ctrl_dt/self.T
        target_position_x = interpolate.splev(virtual_time, self.x_interp, der=0)
        target_position_y = interpolate.splev(virtual_time, self.y_interp, der=0)
        target_position_z = interpolate.splev(virtual_time, self.z_interp, der=0)
        q = np.array([[target_position_x],[target_position_y],[target_position_z]])
        self.ani.draw_goal(q, color=(1.,0.,1.))

        self.ani.draw_quadrotor(self.iris)
        self.ani.draw_goal(self.goal_xyz)
        self.ani.draw_label("Time: {0:.2f}".format(self.t*self.ctrl_dt), 
            (self.ani.window.width // 2, 20.0))
        self.ani.draw()
        time.sleep(0.1)

