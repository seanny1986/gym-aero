import simulation.quadrotor3 as quad
import simulation.config as cfg
import matplotlib.pyplot as pl
import mpl_toolkits.mplot3d.art3d as art3d
import numpy as np
import random
from math import pi, sin, cos
import gym
from gym import error, spaces, utils
from gym.utils import seeding
import math
import simulation.animation_gl as ani_gl


class LandEnv(gym.Env):
    """
        Environment wrapper for training low-level flying skills. The aircraft is required to land
        at a random positon. It is required to lower onto the landing pad, i.e the angle
        between the aircraft and pad is 90 deg.
    """
    def __init__(self):
        metadata = {'render.modes': ['human']}
        self.r_max = 2.5
        self.goal_thresh = 0.05
        self.t = 0
        self.T = 10
        self.action_space = np.zeros((4,))
        self.observation_space = np.zeros((35,))
        self.goal_xyz = np.array([[0.],
                                [0.],
                                [0.]])
        self.spawn = np.array([[random.uniform(-3,3)],
                                [random.uniform(-3,3)],
                                [3.]])
        self.goal_zeta_sin = np.sin(np.array([[0.],
                                            [0.],
                                            [0.]]))
        self.goal_zeta_cos = np.cos(np.array([[0.],
                                            [0.],
                                            [0.]]))
        self.goal_uvw = np.array([[0.],
                                [0.],
                                [0.]])
        self.goal_pqr = np.array([[0.],
                                [0.],
                                [0.]])

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

        # define bounds here
        self.xzy_bound = 0.5
        self.zeta_bound = pi/3.
        self.uvw_bound = 0.5
        self.pqr_bound = 0.25
        self.iris.set_state(self.spawn, np.arcsin(self.goal_zeta_sin), self.goal_uvw, self.goal_pqr)
        self.iris.set_rpm(np.array(self.trim))
        xyz, zeta, uvw, pqr = self.iris.get_state()
        self.vec_xyz = xyz-self.goal_xyz
        self.vec_zeta_sin = np.sin(zeta)-self.goal_zeta_sin
        self.vec_zeta_cos = np.cos(zeta)-self.goal_zeta_cos
        self.vec_uvw = uvw-self.goal_uvw
        self.vec_pqr = pqr-self.goal_pqr
        self.goal_descent_speed = 0.
        self.descent_speed = abs(self.iris.get_inertial_velocity()[2][0])
        self.iner_speed = np.linalg.norm(self.iris.get_inertial_velocity())
        self.dist_norm = np.linalg.norm(self.vec_xyz)
        self.height = xyz[2][0]
        self.att_norm_sin = np.linalg.norm(self.vec_zeta_sin)
        self.att_norm_cos = np.linalg.norm(self.vec_zeta_cos)
        self.vel_norm = np.linalg.norm(self.vec_uvw)
        self.ang_norm = np.linalg.norm(self.vec_pqr)
        self.distance_decrease = False
        self.init_rendering = False

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
        dist_rew = 100.*(self.dist_norm-dist_hat)
        att_rew = 10.*((self.att_norm_sin-att_hat_sin)+(self.att_norm_cos-att_hat_cos))
        vel_rew = 20.*(self.vel_norm-vel_hat)
        ang_rew = 10.*(self.ang_norm-ang_hat)

        #Decent velocity in the Z direction
        d_hat = abs(self.iris.get_inertial_velocity()[2][0])
        descent_rew = 30.*(self.descent_speed -  d_hat)

        #Sum of all velocities
        i_hat = np.linalg.norm(self.iris.get_inertial_velocity())
        iner_rew = 2*(self.iner_speed - i_hat)
        height_rew = 0
        self.iner_speed = i_hat
        self.height =0
        self.descent_speed = d_hat
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
        self.prev_action = action.copy()

        # agent gets a positive reward for time spent in flight
        time_rew = 0.1
        return dist_rew, att_rew, vel_rew, ang_rew, ctrl_rew, time_rew, cmplt_rew, descent_rew, height_rew

    def terminal(self, pos, term):
        xyz, zeta, uvw = pos
        mask1  = 0
        mask2 = 0
        mask3 = False
        #orign = np.array([[0.],
         #               [0.],
          #              [0.]])
        #dist_goal = np.linalg.norm(xyz - self.goal_xyz)

        # Assume aircraft lost due to vortex ring state if body z vel < than -2ms
        if term:
            return True
        elif uvw[2,0] < -2.:
            #print("Aircraft Lost")
            return True
        elif np.sum(mask1) > 0 or np.sum(mask2) > 0 or np.sum(mask3) > 0:
            return True
        elif self.t*self.ctrl_dt >= self.T-self.ctrl_dt:
            #print("Sim time reached: {:.2f}s".format(self.t))
            return True
        else:
            return False

    def step(self, action, term):
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
        #print(action)
        for _ in self.steps:
            xyz, zeta, uvw, pqr = self.iris.step((self.trim_np)+action*self.bandwidth)
        sin_zeta = np.sin(zeta)
        cos_zeta = np.cos(zeta)
        action = (action/self.action_bound[1]).tolist()
        next_pos = xyz.T.tolist()[0]
        next_att = sin_zeta.T.tolist()[0]+cos_zeta.T.tolist()[0]
        next_vel = uvw.T.tolist()[0]+pqr.T.tolist()[0]
        next_state = next_pos+next_att+next_vel
        done = self.terminal((xyz, zeta,uvw), term)
        info = self.reward((xyz, zeta, uvw, pqr), action)
        reward = sum(info)
        pos_goal = self.vec_xyz.T.tolist()[0]
        att_goal = self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]
        vel_goal = self.vec_uvw.T.tolist()[0]+self.vec_pqr.T.tolist()[0]
        descent_goal = [self.descent_speed]
        goals = pos_goal+att_goal+vel_goal+descent_goal
        next_state = next_state+action+goals
        self.t += 1
        return next_state, reward, done, info

    def get_goal(self):
        return self.goal_xyz

    def reset(self):
        self.goal_achieved = False
        self.t = 0
        self.goal_xyz = np.array([[0],
                                [0],
                                [0]])
        self.spawn = np.array([[np.random.randint(-25,25)/10.],
                            [np.random.randint(-25,25)/10.],
                            [3.]])

        self.iris.set_state(self.spawn, np.arcsin(self.goal_zeta_sin), self.goal_uvw, self.goal_pqr)
        self.iris.set_rpm(np.array(self.trim))
        self.descent_speed = abs(self.iris.get_inertial_velocity()[2][0])
        self.iner_speed = np.linalg.norm(self.iris.get_inertial_velocity())
        xyz, zeta, uvw, pqr = self.iris.get_state()
        self.height = xyz[2][0]

        # Resetting all the velocity and distance vectors
        sin_zeta = np.sin(zeta)
        cos_zeta = np.cos(zeta)
        self.vec_xyz = xyz-self.goal_xyz
        self.vec_zeta_sin = sin_zeta
        self.vec_zeta_cos = cos_zeta
        self.vec_zeta_sin = np.sin(zeta)-self.goal_zeta_sin
        self.vec_zeta_cos = np.cos(zeta)-self.goal_zeta_cos
        self.att_norm_sin = np.linalg.norm(self.vec_zeta_sin)
        self.att_norm_cos = np.linalg.norm(self.vec_zeta_cos)
        self.vec_uvw = uvw-self.goal_uvw
        self.vec_pqr = pqr-self.goal_pqr
        self.vel_norm = np.linalg.norm(self.vec_uvw)
        self.ang_norm = np.linalg.norm(self.vec_pqr)
        self.dist_norm = np.linalg.norm(self.vec_xyz)
        self.vec_uvw = uvw
        self.vec_pqr = pqr
        action = (self.trim_np/self.action_bound[1]).tolist()
        pos_goal = self.vec_xyz.T.tolist()[0]
        att_goal = self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]
        vel_goal = self.vec_uvw.T.tolist()[0]+self.vec_pqr.T.tolist()[0]
        descent_goal = [self.descent_speed]
        goals = pos_goal+att_goal+vel_goal+descent_goal
        pos = xyz.T.tolist()[0]
        att = sin_zeta.T.tolist()[0]+cos_zeta.T.tolist()[0]
        vel = uvw.T.tolist()[0]+pqr.T.tolist()[0]
        state = pos+att+vel+action+goals
        return state

    def render(self, mode='human', close=False):
        if not self.init_rendering:
            self.ani = ani_gl.VisualizationGL(name="Land")
            self.init_rendering = True
        self.ani.draw_quadrotor(self.iris)
        self.ani.draw_goal(self.goal_xyz)
        self.ani.draw_label("Time: {0:.2f}".format(self.t*self.ctrl_dt), 
            (self.ani.window.width // 2, 20.0))
        self.ani.draw()