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

class TrajectoryEnv(gym.Env):
    """
        Environment wrapper for training low-level flying skills. In this environment, the aircraft
        has a deterministic starting state by default. We can switch it to have non-deterministic 
        initial states. This is obviously much harder.
    """
    def __init__(self):
        metadata = {'render.modes': ['human']}
        self.r = 1.
        self.goal_thresh = 0.05
        self.t = 0
        self.T = 6
        self.action_space = np.zeros((4,))
        self.observation_space = np.zeros((31,))

        # build list of waypoints for the aircraft to fly to
        self.traj_len = 4
        self.goal_list = []
        x = np.array([[0.],[0.],[0.]])
        for i in range(self.traj_len):
            x += self.generate_goal()
            self.goal_list.append(x)

        self.goal_curr = 0
        self.goal_next_curr = self.goal_curr+1
        self.goal_xyz = self.goal_list[self.goal_curr]
        self.goal_xyz_next = self.goal_list[self.goal_next_curr]
        self.goal_zeta_sin = np.sin(np.array([[0.],
                                            [0.],
                                            [0.]]))
        self.goal_zeta_cos = np.cos(np.array([[0.],
                                            [0.],
                                            [0.]]))
        self.datum = np.array([[0.],[0.],[0.]])

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

        xyz, zeta, uvw, pqr = self.iris.get_state()

        self.vec_xyz = xyz-self.goal_xyz
        self.vec_zeta_sin = np.sin(zeta)-self.goal_zeta_sin
        self.vec_zeta_cos = np.cos(zeta)-self.goal_zeta_cos

        self.dist_norm = np.linalg.norm(self.vec_xyz)
        self.att_norm_sin = np.linalg.norm(self.vec_zeta_sin)
        self.att_norm_cos = np.linalg.norm(self.vec_zeta_cos)

        self.fig = None
        self.axis3d = None

    def get_goal(self):
        return self.goal_xyz

    def reward(self, state, action):
        xyz, zeta, uvw, pqr = state
        s_zeta = np.sin(zeta)
        c_zeta = np.cos(zeta)
        curr_dist = xyz-self.goal_xyz
        curr_att_sin = s_zeta-self.goal_zeta_sin
        curr_att_cos = c_zeta-self.goal_zeta_cos
        
        # magnitude of the distance from the goal 
        dist_hat = np.linalg.norm(curr_dist)
        att_hat_sin = np.linalg.norm(curr_att_sin)
        att_hat_cos = np.linalg.norm(curr_att_cos)

        # agent gets a negative reward based on how far away it is from the desired goal state
        dist_rew = 100*(self.dist_norm-dist_hat)
        att_rew = 10*((self.att_norm_sin-att_hat_sin)+(self.att_norm_cos-att_hat_cos))

        self.dist_norm = dist_hat
        self.att_norm_sin = att_hat_sin
        self.att_norm_cos = att_hat_cos

        self.vec_xyz = curr_dist
        self.vec_zeta_sin = curr_att_sin
        self.vec_zeta_cos = curr_att_cos

        if self.dist_norm <= self.goal_thresh:
            cmplt_rew = 100.
            self.goal_achieved(xyz)
            curr_dist = xyz-self.goal_xyz
            dist_hat = np.linalg.norm(curr_dist)
            self.dist_norm = dist_hat
        else:
            cmplt_rew = 0

        # agent gets a negative reward for excessive action inputs
        ctrl_rew = -np.sum(((action/self.action_bound[1])**2))

        # agent gets a positive reward for time spent in flight
        time_rew = -0.1
        return dist_rew, att_rew, ctrl_rew, time_rew, cmplt_rew

    def terminal(self, pos):
        xyz, zeta = pos
        mask3 = self.dist_norm > 5.
        if np.sum(mask3) > 0:
            return True
        elif (self.dist_norm <= self.goal_thresh) and (self.goal_curr == self.traj_len-1):
            print("Last goal achieved!")
            return True
        elif self.t >= self.T:
            print("Sim time reached: {:.2f}s".format(self.t))
            return True
        else:
            return False
    
    def goal_achieved(self, xyz):
        self.datum = xyz.copy()
        self.goal_curr += 1
        self.goal_next_curr += 1
        self.goal_xyz = self.goal_list[self.goal_curr]
        if self.goal_next_curr >= len(self.goal_list):
            self.goal_xyz_next = np.array([[0.],[0.],[0.]])
        else:
            self.goal_xyz_next = self.goal_list[self.goal_xyz_next]

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
            xs, zeta, uvw, pqr = self.iris.step(self.trim_np+action*self.bandwidth)
        xyz = xs.copy()-self.datum.copy()
        self.t += self.ctrl_dt
        sin_zeta = np.sin(zeta)
        cos_zeta = np.cos(zeta)
        a = (action/self.action_bound[1]).tolist()
        next_state = xyz.T.tolist()[0]+sin_zeta.T.tolist()[0]+cos_zeta.T.tolist()[0]+uvw.T.tolist()[0]+pqr.T.tolist()[0]
        info = self.reward((xyz, zeta, uvw, pqr), action)
        done = self.terminal((xyz, zeta))
        reward = sum(info)
        goals = self.vec_xyz.T.tolist()[0]+self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]
        next_goal = self.goal_xyz_next.T.tolist()[0]
        next_state = next_state+a+goals+next_goal
        return next_state, reward, done, info

    def reset(self):
        self.t = 0.
        xyz, zeta, uvw, pqr = self.iris.reset()
        self.iris.set_rpm(np.array(self.trim))
        self.goal_list = []
        x = np.array([[0.],[0.],[0.]])
        for i in range(self.traj_len):
            x += self.generate_goal()
            self.goal_list.append(x)
        sin_zeta = np.sin(zeta)
        cos_zeta = np.cos(zeta)
        self.vec_xyz = xyz-self.goal_xyz
        self.vec_zeta_sin = sin_zeta
        self.vec_zeta_cos = cos_zeta
        a = (self.trim_np/self.action_bound[1]).tolist()
        goals = self.vec_xyz.T.tolist()[0]+self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]
        next_goal = self.goal_xyz_next.T.tolist()[0]
        state = xyz.T.tolist()[0]+sin_zeta.T.tolist()[0]+cos_zeta.T.tolist()[0]+uvw.T.tolist()[0]+pqr.T.tolist()[0]
        return state+a+goals+next_goal

    def generate_goal(self):
        x = np.random.uniform(low=-1, high=1, size=(3,1))
        x_hat = x/np.linalg.norm(x)
        xyz = self.r*x_hat
        return xyz
    
    def render(self, mode='human', close=False):
        if self.fig is None:
            # rendering parameters
            pl.close("all")
            pl.ion()
            self.fig = pl.figure("Flying Skills")
            self.axis3d = self.fig.add_subplot(111, projection='3d')
            self.vis = ani.Visualization(self.iris, 6, quaternion=True)
        pl.figure("Flying Skills")
        self.axis3d.cla()
        self.vis.draw3d_quat(self.axis3d)
        for i, g in enumerate(self.goal_list):
            if i == 0:
                self.vis.draw_line(self.axis3d,np.array([0.,0.,0.]),self.goal_list[i], color='r')
            else:
                self.vis.draw_line(self.axis3d,self.goal_list[i-1], self.goal_list[i], color='r')
            self.vis.draw_goal(self.axis3d, g)
        self.axis3d.set_xlim(-3, 3)
        self.axis3d.set_ylim(-3, 3)
        self.axis3d.set_zlim(-3, 3)
        self.axis3d.set_xlabel('West/East [m]')
        self.axis3d.set_ylabel('South/North [m]')
        self.axis3d.set_zlabel('Down/Up [m]')
        self.axis3d.set_title("Time %.3f s" %(self.t))
        pl.pause(0.001)
        pl.draw()


