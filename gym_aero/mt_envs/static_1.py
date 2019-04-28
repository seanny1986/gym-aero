import simulation.quadrotor3 as quad
import simulation.config as cfg
import numpy as np
import random
from math import pi, sin, cos
import gym
from gym import error, spaces, utils
from gym.utils import seeding
import simulation.animation_gl as ani_gl


"""
    Environment wrapper for a climb & hover task. The goal of this task is for the agent to climb from [0, 0, 0]^T
    to [0, 0, 1.5]^T, and to remain at that altitude until the the episode terminates at T=15s.
"""

class OneEnv(gym.Env):
    def __init__(self):
        metadata = {'render.modes': ['human']}
        
        # environment parameters
        self.goal_xyz = np.array([[1.5],
                                [0.],
                                [0.]])
        self.goal_zeta_sin = np.sin(np.zeros((3,1)))
        self.goal_zeta_cos = np.cos(np.zeros((3,1)))
        self.goal_uvw = np.zeros((3,1))
        self.goal_pqr = np.zeros((3,1))
        self.goal_thresh = 0.1
        self.t = 0
        self.T = 3
        self.action_space = np.zeros((4,))
        self.observation_space = np.zeros((19,))

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

        self.init_rendering = False

    def get_goal(self):
        return self.goal_xyz

    def reward(self, state, action):
        xyz, zeta, uvw, pqr = state
        s_zeta = np.sin(zeta)
        c_zeta = np.cos(zeta)
        curr_dist = xyz-self.goal_xyz
        curr_att_sin = s_zeta-self.goal_zeta_sin
        curr_att_cos = c_zeta-self.goal_zeta_cos
        curr_vel = uvw-self.goal_uvw
        curr_ang = pqr-self.goal_pqr
        
        # magnitude of the distance from the goal 
        dist_hat = np.linalg.norm(curr_dist)
        att_hat_sin = np.linalg.norm(curr_att_sin)
        att_hat_cos = np.linalg.norm(curr_att_cos)
        vel_hat = np.linalg.norm(curr_vel)
        ang_hat = np.linalg.norm(curr_ang)

        # agent gets a negative reward based on how far away it is from the desired goal state
        dist_rew = 100*(self.dist_norm-dist_hat)
        att_rew = 10*((self.att_norm_sin-att_hat_sin)+(self.att_norm_cos-att_hat_cos))
        vel_rew = 0.1*(self.vel_norm-vel_hat)
        ang_rew = 0.1*(self.ang_norm-ang_hat)
        self.dist_norm = dist_hat
        self.att_norm_sin = att_hat_sin
        self.att_norm_cos = att_hat_cos
        self.vel_norm = vel_hat
        self.ang_norm = ang_hat
        self.vec_xyz = curr_dist
        self.vec_zeta_sin = curr_att_sin
        self.vec_zeta_cos = curr_att_cos
        self.vec_uvw = curr_vel
        self.vec_pqr = curr_ang
        
        if self.dist_norm <= self.goal_thresh:
            cmplt_rew = 100.
        else:
            cmplt_rew = 0
        
        # agent gets a negative reward for excessive action inputs
        ctrl_rew = 0.
        ctrl_rew -= np.sum(((action-self.trim_np)/self.action_bound[1])**2)
        ctrl_rew -= np.sum((((action-self.prev_action)/self.action_bound[1])**2))
        ctrl_rew -= 10.*np.linalg.norm(uvw-self.prev_uvw)
        ctrl_rew -= 10.*np.linalg.norm(pqr-self.prev_pqr)
        
        # agent gets a positive reward for time spent in flight
        time_rew = 0.
        
        return dist_rew, att_rew, vel_rew, ang_rew, ctrl_rew, time_rew, cmplt_rew

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
        mask3 = self.dist_norm > 5.
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
        for _ in self.steps:
            xyz, zeta, uvw, pqr = self.iris.step(self.trim_np+action*self.bandwidth)
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
#        position_goal = self.vec_xyz.T.tolist()[0]
#        attitude_goal = self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]
#        velocity_goal = self.vec_uvw.T.tolist()[0]+self.vec_pqr.T.tolist()[0]
#        goals = position_goal+attitude_goal+velocity_goal
        next_state = next_state+current_rpm
        self.prev_action = action.copy()
        self.prev_uvw = uvw.copy()
        self.prev_pqr = pqr.copy()
        self.t += 1
        return next_state, reward, done, {"dist_rew": info[0], 
                                        "att_rew": info[1], 
                                        "vel_rew": info[2], 
                                        "ang_rew": info[3], 
                                        "ctrl_rew": info[4], 
                                        "time_rew": info[5]}

    def reset(self):
        self.t = 0
        xyz, zeta, uvw, pqr = self.iris.reset()
        self.iris.set_rpm(np.array(self.trim))
        sin_zeta = np.sin(zeta)
        cos_zeta = np.cos(zeta)
        current_rpm = (self.iris.get_rpm()/self.action_bound[1]).tolist()
        next_position = xyz.T.tolist()[0]
        next_attitude = sin_zeta.T.tolist()[0]+cos_zeta.T.tolist()[0]
        next_velocity = uvw.T.tolist()[0]+pqr.T.tolist()[0]  
        next_state = next_position+next_attitude+next_velocity
        self.vec_xyz = xyz-self.goal_xyz
        self.vec_zeta_sin = np.sin(zeta)-self.goal_zeta_sin
        self.vec_zeta_cos = np.cos(zeta)-self.goal_zeta_cos
        self.vec_uvw = uvw-self.goal_uvw
        self.vec_pqr = pqr-self.goal_pqr
        self.dist_norm = np.linalg.norm(self.vec_xyz)
        self.att_norm_sin = np.linalg.norm(self.vec_zeta_sin)
        self.att_norm_cos = np.linalg.norm(self.vec_zeta_cos)
        self.vel_norm = np.linalg.norm(self.vec_uvw)
        self.ang_norm = np.linalg.norm(self.vec_pqr)
#        position_goal = self.vec_xyz.T.tolist()[0] 
#        attitude_goal = self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]
#        velocity_goal = self.vec_uvw.T.tolist()[0]+self.vec_pqr.T.tolist()[0]
#        goals = position_goal+attitude_goal+velocity_goal
        next_state = next_state+current_rpm
        self.prev_action = self.trim_np.copy()
        self.prev_uvw = np.array([[0.],[0.],[0.]])
        self.prev_pqr = np.array([[0.],[0.],[0.]])
        return next_state
    
    def render(self, mode='human', close=False):
        if not self.init_rendering:
            self.ani = ani_gl.VisualizationGL(name="Static Waypoint")
            self.init_rendering = True
        self.ani.draw_quadrotor(self.iris)
        self.ani.draw_goal(self.goal_xyz)
        self.ani.draw_label("Time: {0:.2f}".format(self.t*self.ctrl_dt), 
            (self.ani.window.width // 2, 20.0))
        self.ani.draw()


