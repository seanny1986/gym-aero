import simulation.quadrotor3 as quad
import simulation.config as cfg
import matplotlib.pyplot as pl
import numpy as np
import random
from math import pi, sin, cos
import gym
from gym import error, spaces, utils
from gym.utils import seeding
import simulation.animation_gl as ani_gl

class RecoveryEnv(gym.Env):
    """
        Environment wrapper for training aircraft recovery. In this environment, the aircraft is
        initialized with a bounded, random starting state, and must recover to a given goal point.
        This is similar to the random waypoint task, but much harder, since the aircraft starts from
        a random state that may not be recoverable.
    """
    def __init__(self):
        metadata = {'render.modes': ['human']}
        self.r_max = 2.5
        self.goal_thresh = 0.05
        self.t = 0
        self.T = 3.
        self.action_space = np.zeros((4,))
        self.observation_space = np.zeros((34,))

        # goal position and attitude in the inertial frame
        self.goal_xyz = np.array([[0.],
                                [0.],
                                [0.]])
        self.goal_zeta_sin = np.sin(np.array([[0.],
                                            [0.],
                                            [0.]]))
        self.goal_zeta_cos = np.cos(np.array([[0.],
                                            [0.],
                                            [0.]]))

        # the velocity of the aircraft in the inertial frame is probably a better metric here, but
        # since our goal state is (0,0,0), this should be fine.
        self.goal_uvw = np.array([[0.],
                                [0.],
                                [0.]])
        self.goal_pqr = np.array([[0.],
                                [0.],
                                [0.]])

        # initialize simulation
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

        # generate random state for the quadrotor, set aircraft state
        self.v_max = 1.
        self.x_max = 0.
        self.zeta_max = pi
        self.omega_max = 1.*pi
        xyz, zeta, uvw, pqr = self.generate_s0()
        self.iris.set_state(xyz, zeta, uvw, pqr)

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
        self.init_rendering = False
        self.lazy_action = False
        self.lazy_change = False

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
        dist_rew = 0.*(self.dist_norm-dist_hat)
        att_rew = 100*((self.att_norm_sin-att_hat_sin)+(self.att_norm_cos-att_hat_cos))
        vel_rew = 50*(self.vel_norm-vel_hat)
        ang_rew = 50*(self.ang_norm-ang_hat)
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
        if self.lazy_action:
            ctrl_rew -= np.sum(((action-self.trim_np)/self.action_bound[1])**2)
        if self.lazy_change:
            ctrl_rew -= np.sum((((action-self.prev_action)/self.action_bound[1])**2))
        self.prev_action = action.copy()
        
        # agent gets a positive reward for time spent in flight
        time_rew = 0.1
        
        return dist_rew, att_rew, vel_rew, ang_rew, ctrl_rew, time_rew, cmplt_rew

    def terminal(self, pos):
        xyz, zeta = pos
        mask1 = 0
        mask2 = 0
        mask3 = self.dist_norm > 10.
        if np.sum(mask1) > 0 or np.sum(mask2) > 0 or np.sum(mask3) > 0:
            return True
        #elif self.dist_norm <= self.goal_thresh:
        #    print("Goal Achieved!")
        #    return True
        elif self.t >= self.T:
            #print("Sim time reached: {:.2f}s".format(self.t))
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
        position_goal = self.vec_xyz.T.tolist()[0]
        attitude_goal = self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]
        velocity_goal = self.vec_uvw.T.tolist()[0]+self.vec_pqr.T.tolist()[0]
        goals = position_goal+attitude_goal+velocity_goal
        next_state = next_state+current_rpm+goals
        self.t += 1
        return next_state, reward, done, {"dist_rew": info[0], 
                                        "att_rew": info[1], 
                                        "vel_rew": info[2], 
                                        "ang_rew": info[3], 
                                        "ctrl_rew": info[4], 
                                        "time_rew": info[5], 
                                        "cmplt_rew": info[6]}

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
        xyz, zeta, uvw, pqr = self.goal_xyz = self.generate_s0()
        self.iris.set_rpm(np.array(self.trim))
        self.iris.set_state(xyz, zeta, uvw, pqr)
        sin_zeta = np.sin(zeta)
        cos_zeta = np.cos(zeta)
        current_rpm = (self.iris.get_rpm()/self.action_bound[1]).tolist()
        next_position = xyz.T.tolist()[0]
        next_attitude = sin_zeta.T.tolist()[0]+cos_zeta.T.tolist()[0]
        next_velocity = uvw.T.tolist()[0]+pqr.T.tolist()[0]  
        next_state = next_position+next_attitude+next_velocity
        self.vec_xyz = xyz-self.goal_xyz
        self.vec_zeta_sin = sin_zeta
        self.vec_zeta_cos = cos_zeta
        self.vec_uvw = uvw
        self.vec_pqr = pqr
        self.dist_norm = np.linalg.norm(self.vec_xyz)
        self.att_norm_sin = np.linalg.norm(self.vec_zeta_sin)
        self.att_norm_cos = np.linalg.norm(self.vec_zeta_cos)
        self.vel_norm = np.linalg.norm(self.vec_uvw)
        self.ang_norm = np.linalg.norm(self.vec_pqr)
        position_goal = self.vec_xyz.T.tolist()[0]
        attitude_goal = self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]
        velocity_goal = self.vec_uvw.T.tolist()[0]+self.vec_pqr.T.tolist()[0]
        goals = position_goal+attitude_goal+velocity_goal
        next_state = next_state+current_rpm+goals
        return next_state

    def generate_s0(self):
        # generate random unit vector for linear and angular velocities
        zeta_hat = np.random.uniform(low=-1, high=1, size=(3,1))
        uvw_hat = np.random.uniform(low=-1, high=1, size=(3,1))
        pqr_hat = np.random.uniform(low=-1, high=1, size=(3,1))
        xyz_hat = np.array([[0.],[0.],[0.]])
        zeta_hat = zeta_hat/np.linalg.norm(zeta_hat)
        uvw_hat = uvw_hat/np.linalg.norm(uvw_hat)
        pqr_hat = pqr_hat/np.linalg.norm(pqr_hat)
        xyz = self.x_max*xyz_hat
        zeta = self.zeta_max*zeta_hat
        uvw = self.v_max*uvw_hat
        pqr = self.omega_max*pqr_hat
        return xyz, zeta, uvw, pqr
    
    def render(self, mode='human', close=False):
        """
        Parameters
        ----------
        mode :
        close :

        Returns
        -------
            n/a
        """

        if not self.init_rendering:
            self.ani = ani_gl.VisualizationGL(name="RandomWaypoint")
            self.init_rendering = True
        self.ani.draw_quadrotor(self.iris)
        self.ani.draw_goal(self.goal_xyz)
        self.ani.draw_label("Time: {0:.2f}".format(self.t*self.ctrl_dt), 
            (self.ani.window.width // 2, 20.0))
        self.ani.draw()


