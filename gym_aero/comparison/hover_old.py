import old_simulation.quadrotor3 as quad
import old_simulation.config as cfg
import old_simulation.animation_gl as ani_gl
import numpy as np
import random
from math import pi, sin, cos
import gym
from gym import error, spaces, utils
from gym.utils import seeding
import time


class HoverOld(gym.Env):
    """
    Environment wrapper for a sustained hover task. The goal of this task is for the agent to 
    hover at the point [0, 0, 0]^T until the the episode terminates at T=15s.
    -- Sean Morrison
    """
    
    def __init__(self):
        metadata = {'render.modes': ['human']}

        print()
        print("Running old environment")

        # environment parameters
        self.goal_xyz = np.array([[0.],
                                [0.],
                                [0.]])
        self.goal_zeta_sin = np.sin(np.array([[0.],
                                            [0.],
                                            [0.]]))
        self.goal_zeta_cos = np.cos(np.array([[0.],
                                            [0.],
                                            [0.]]))
        self.goal_uvw = np.array([[0.], ##XYZ of actual quad
                                [0.],
                                [0.]])
        self.goal_pqr = np.array([[0.], ##PQR
                                [0.],
                                [0.]])

        self.t = 0
        self.T = 15
        self.action_space = np.zeros((4,))
        self.observation_space = np.zeros((34,))

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
        self.prev_action = self.trim_np.copy()
        self.bandwidth = 35.
        self.iris.set_state(self.goal_xyz, np.arcsin(self.goal_zeta_sin), self.goal_uvw, self.goal_pqr)
        xyz, zeta, uvw, pqr = self.iris.get_state()
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

        self.prev_uvw = np.array([[0.],[0.],[0.]])
        self.prev_pqr = np.array([[0.],[0.],[0.]])

        self.init_rendering = False

    def set_action_frequency(self, ctrl_dt):
        self.ctrl_dt = ctrl_dt
        self.steps = range(int(self.ctrl_dt/self.sim_dt))

    def get_goal(self):
        """
        Parameters
        ----------
        n/a
        Returns
        -------
            goal_xyz (numpy array):
                a 3x1 numpy array of the aircraft's goal position in Euclidean coordinates
        """

        return self.goal_xyz
    
    def get_data(self):
        xyz = self.iris.state[:3,:]
        xyz = xyz.flatten().tolist()
        q = self.iris.state[3:7,:]
        zeta = self.iris.q_to_euler(q)
        zeta = zeta.flatten().tolist()
        uvw = self.iris.state[7:10,:]
        uvw = uvw.flatten().tolist()
        pqr = self.iris.state[10:,:]
        pqr = pqr.flatten().tolist()
        return xyz, zeta, uvw, pqr

    def reward(self, state, action):
        """
        Parameters
        ----------
        state :
        action :
        Returns
        -------
            dist_rew (float) : 
                a float reward value based on the change in distance to the goal position.
                This reward is positive when the aircraft moves closer, and negative when
                it moves further away.
            att_rew (float) : 
                a float reward value based on the change in distance to the goal attitude.
                This reward is positive when the aircraft moves towards the goal attitude, 
                and negative when it moves away from it.
            vel_rew (float) : 
                a float reward value based on the change in distance to the goal velocity.
                This reward is positive when the aircraft moves towards the goal velocity, 
                and negative when it moves away from it.
            ang_rew (float) : 
                a float reward value based on the change in distance to the goal angular
                velocity. This reward is positive when the aircraft moves towards the goal 
                angular velocity, and negative when it moves away from it.
            ctrl_rew (float) : 
                a float reward value that penalizes the aircraft for taking large actions.
                In particular, we want to minimize both the distance from the "expected"
                action, as well as the change in the action between two timesteps.
            time_rew (float) : 
                a float reward value based on time. In tasks where we want the aircraft to
                fly for a long period of time, this should be positive. In tasks where we
                want the aircraft to maximize speed (minimize flight time), this should be 
                negative.
            cmplt_rew (float) : 
                a constant reward value for completing the task.
        """

        xyz, zeta, uvw, pqr = state
        s_zeta = np.sin(zeta)
        c_zeta = np.cos(zeta)
        curr_dist = xyz-self.goal_xyz
        curr_att_sin = s_zeta-self.goal_zeta_sin
        curr_att_cos = c_zeta-self.goal_zeta_cos
        curr_vel = uvw-self.goal_uvw
        curr_ang = pqr-self.goal_pqr

        #print("old")
        #print(self.dist_norm)
        #print(self.att_norm_sin)
        #print(self.att_norm_cos)
        #print(self.vel_norm)
        #print(self.ang_norm)
        #print("old action: ", action)

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

        # agent gets a negative reward for excessive action inputs
        ctrl_rew = 0.
        ctrl_rew -= np.sum(((action-self.trim_np)/self.action_bound[1])**2)
        ctrl_rew -= np.sum((((action-self.prev_action)/self.action_bound[1])**2))
        ctrl_rew -= 10.*np.sum((uvw-self.prev_uvw)**2)
        ctrl_rew -= 10.*np.sum((pqr-self.prev_pqr)**2)

        # agent gets a positive reward for time spent in flight
        time_rew = 0.
        return dist_rew, att_rew, vel_rew, ang_rew, ctrl_rew, time_rew

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
        mask = self.dist_norm > 5
        if np.sum(mask) > 0:
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
        info = self.reward((xyz, zeta, uvw, pqr), self.trim_np+action*self.bandwidth)
        done = self.terminal((xyz, zeta))
        reward = sum(info)
        position_goal = self.vec_xyz.T.tolist()[0] 
        attitude_goal = self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]
        velocity_goal = self.vec_uvw.T.tolist()[0]+self.vec_pqr.T.tolist()[0]
        goals = position_goal+attitude_goal+velocity_goal
        next_state = next_state+goals+current_rpm
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
        """
        Parameters
        ----------
        n/a
        Returns
        -------
        next_state
            next_state (list) :
                a list of float values containing the state (position, attitude, and
                velocity), the current rpm of the vehicle, and the aircraft's goals
                (position, attitude, velocity).
        """

        self.t = 0
        xyz, zeta, uvw, pqr = self.iris.reset()
        self.iris.set_rpm(np.array(self.trim))
        self.prev_action = self.trim_np.copy()
        self.prev_uvw = np.array([[0.],[0.],[0.]])
        self.prev_pqr = np.array([[0.],[0.],[0.]])
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
        position_goal = self.vec_xyz.T.tolist()[0] 
        attitude_goal = self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]
        velocity_goal = self.vec_uvw.T.tolist()[0]+self.vec_pqr.T.tolist()[0]
        goals = position_goal+attitude_goal+velocity_goal
        next_state = next_state+goals+current_rpm
        return next_state

    def render(self, mode='human', close=False):
        if not self.init_rendering:
            self.ani = ani_gl.VisualizationGL(name="Hover")
            self.init_rendering = True
        self.ani.draw_quadrotor(self)
        self.ani.draw_goal(self.goal_xyz.flatten().tolist())
        self.ani.draw_label("Time: {0:.2f}".format(self.t*self.ctrl_dt), 
            (self.ani.window.width // 2, 20.0))
        self.ani.draw()