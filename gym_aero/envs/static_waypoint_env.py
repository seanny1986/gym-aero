import simulation.quadrotor3 as quad
import simulation.config as cfg
import simulation.animation as ani
import matplotlib.pyplot as pl
import numpy as np
import random
from math import pi, sin, cos
import gym
from gym import error, spaces, utils
from gym.utils import seeding


"""
    Environment wrapper for a climb & hover task. The goal of this task is for the agent to climb from [0, 0, 0]^T
    to [0, 0, 1.5]^T, and to remain at that altitude until the the episode terminates at T=15s.
"""

class StaticWaypointEnv(gym.Env):
    def __init__(self):
        metadata = {'render.modes': ['human']}
        
        # environment parameters
        self.goal_xyz = np.array([[1.5],
                                [0.],
                                [0.]])
        self.goal_zeta_sin = np.sin(np.array([[0.],
                                            [0.],
                                            [0.]]))
        self.goal_zeta_cos = np.cos(np.array([[0.],
                                            [0.],
                                            [0.]]))
        self.goal_pqr = np.array([[0.],
                                [0.],
                                [0.]])
        self.goal_thresh = 0.05
        self.t = 0
        self.T = 5
        self.action_space = np.zeros((4,))
        self.observation_space = np.zeros((31,))

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

        self.iris.set_state(self.goal_xyz, np.arcsin(self.goal_zeta_sin), np.array([[0.],[0.],[0.]]), np.array([[0.],[0.],[0.]]))
        xyz, zeta, _, pqr = self.iris.get_state()

        self.vec_xyz = xyz-self.goal_xyz
        self.vec_zeta_sin = np.sin(zeta)-self.goal_zeta_sin
        self.vec_zeta_cos = np.cos(zeta)-self.goal_zeta_cos
        self.vec_pqr = pqr-self.goal_pqr

        self.dist_norm = np.linalg.norm(self.vec_xyz)
        self.att_norm_sin = np.linalg.norm(self.vec_zeta_sin)
        self.att_norm_cos = np.linalg.norm(self.vec_zeta_cos)
        self.ang_norm = np.linalg.norm(self.vec_pqr)

        self.fig = None
        self.axis3d = None
        self.v = None

    def reward(self, state, action):
        xyz, zeta, _, pqr = state
        
        s_zeta = np.sin(zeta)
        c_zeta = np.cos(zeta)

        curr_dist = xyz-self.goal_xyz
        curr_att_sin = s_zeta-self.goal_zeta_sin
        curr_att_cos = c_zeta-self.goal_zeta_cos
        curr_ang = pqr-self.goal_pqr
        
        dist_hat = np.linalg.norm(curr_dist)
        att_hat_sin = np.linalg.norm(curr_att_sin)
        att_hat_cos = np.linalg.norm(curr_att_cos)
        ang_hat = np.linalg.norm(curr_ang)

        # agent gets a negative reward based on how far away it is from the desired goal state
        if dist_hat > self.goal_thresh:
            dist_rew = 1/dist_hat
        else:
            dist_rew = 1/self.goal_thresh
        att_rew = 0*((self.att_norm_sin-att_hat_sin)+(self.att_norm_cos-att_hat_cos))
        ang_rew = 0*(self.ang_norm-ang_hat)
        if dist_hat < 0.05:
            dist_rew += 0
        
        self.dist_norm = dist_hat
        self.att_norm_sin = att_hat_sin
        self.att_norm_cos = att_hat_cos
        self.ang_norm = ang_hat

        self.vec_xyz = curr_dist
        self.vec_zeta_sin = curr_att_sin
        self.vec_zeta_cos = curr_att_cos
        self.vec_pqr = curr_ang

        ctrl_rew = 0#-np.sum(((action/self.action_bound[1])**2))
        time_rew = 0#1.
        return dist_rew, att_rew, ang_rew, ctrl_rew, time_rew

    def terminal(self, pos):
        xyz, zeta = pos
        mask1 = 0#zeta[0:2] > pi/2
        mask2 = 0#zeta[0:2] < -pi/2
        mask3 = self.dist_norm > 2
        if np.sum(mask1) > 0 or np.sum(mask2) > 0 or np.sum(mask3) > 0:
            return True
        #elif self.goal_achieved:
            #print("Goal Achieved!")
        #    return True
        elif self.t == self.T:
            print("Sim time reached")
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
            xyz, zeta, uvw, pqr = self.iris.step(action)
        sin_zeta = np.sin(zeta)
        cos_zeta = np.cos(zeta)
        a = (action/self.action_bound[1]).tolist()
        next_state = xyz.T.tolist()[0]+sin_zeta.T.tolist()[0]+cos_zeta.T.tolist()[0]+uvw.T.tolist()[0]+pqr.T.tolist()[0]
        info = self.reward((xyz, zeta, uvw, pqr), action)
        done = self.terminal((xyz, zeta))
        reward = sum(info)
        goals = self.vec_xyz.T.tolist()[0]+self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]+self.vec_pqr.T.tolist()[0]
        next_state = [next_state+a+goals]
        self.t += self.ctrl_dt
        return next_state, reward, done, info

    def reset(self):
        self.t = 0.
        self.iris.set_state(np.array([[0.],[0.],[0.]]), np.sin(self.goal_zeta_sin), np.array([[0.],[0.],[0.]]), np.array([[0.],[0.],[0.]]))
        xyz, zeta, uvw, pqr = self.iris.get_state()
        self.iris.set_rpm(np.array(self.trim))
        sin_zeta = np.sin(zeta)
        cos_zeta = np.cos(zeta)
        self.vec_xyz = xyz-self.goal_xyz
        self.vec_zeta_sin = sin_zeta-self.goal_zeta_sin
        self.vec_zeta_cos = cos_zeta-self.goal_zeta_cos
        a = [x/self.action_bound[1] for x in self.trim]
        goals = self.vec_xyz.T.tolist()[0]+self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]+self.vec_pqr.T.tolist()[0]
        state = [xyz.T.tolist()[0]+sin_zeta.T.tolist()[0]+cos_zeta.T.tolist()[0]+uvw.T.tolist()[0]+pqr.T.tolist()[0]+a+goals]
        return state
    
    def render(self, mode='human', close=False):
        if self.fig is None:
            # rendering parameters
            pl.close("all")
            pl.ion()
            self.fig = pl.figure("Hover")
            self.axis3d = self.fig.add_subplot(111, projection='3d')
            self.vis = ani.Visualization(self.iris, 6, quaternion=True)
            
        pl.figure("Hover")
        self.axis3d.cla()
        self.vis.draw3d_quat(self.axis3d)
        self.vis.draw_goal(self.axis3d, self.goal_xyz)
        self.axis3d.set_xlim(-3, 3)
        self.axis3d.set_ylim(-3, 3)
        self.axis3d.set_zlim(-3, 3)
        self.axis3d.set_xlabel('West/East [m]')
        self.axis3d.set_ylabel('South/North [m]')
        self.axis3d.set_zlabel('Down/Up [m]')
        self.axis3d.set_title("Time %.3f s" %(self.t))
        pl.pause(0.001)
        pl.draw()


