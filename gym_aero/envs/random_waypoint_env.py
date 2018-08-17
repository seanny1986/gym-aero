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

class RandomWaypointEnv(gym.Env):
    """
        Environment wrapper for training low-level flying skills. In this environment, the aircraft
        has a deterministic starting state by default. We can switch it to have non-deterministic
        initial states. This is obviously much harder.
    """
    def __init__(self):
        metadata = {'render.modes': ['human']}

        # environment parameters
        self.deterministic_s0 = True
        self.goal = self.generate_goal(0.5)
        self.goal_thresh = 0.1
        self.t = 0
        self.T = 2.5
        self.r = 1.5
        self.action_space = np.zeros((4,))
        self.observation_space = np.zeros((22,))

        # simulation parameters
        self.params = cfg.params
        self.iris = quad.Quadrotor(self.params)
        self.ctrl_dt = 0.05
        self.sim_dt = self.params["dt"]
        self.steps = range(int(self.ctrl_dt/self.sim_dt))
        self.hov_rpm = self.iris.hov_rpm
        self.trim = [self.hov_rpm, self.hov_rpm,self.hov_rpm, self.hov_rpm]
        self.action_bound = [0, self.iris.max_rpm]
        self.H = int(self.T/self.ctrl_dt)

        # define bounds here
        self.xzy_bound = 0.5
        self.zeta_bound = pi/3
        self.uvw_bound = 0.5
        self.pqr_bound = 0.25

        xyz, _, _, _ = self.iris.get_state()

        self.vec = xyz-self.goal
        self.dist_norm = np.linalg.norm(self.vec)

        self.fig = None
        self.axis3d = None
        self.v = None

    def deterministic_start(self, deterministic_s0):
        self.deterministic_s0 = deterministic_s0

    def generate_s0(self):
        xyz = np.random.uniform(low=-self.xzy_bound, high=self.xzy_bound, size=(3,1))
        zeta = np.random.uniform(low=-self.zeta_bound, high=self.zeta_bound, size=(3,1))
        uvw = np.random.uniform(low=-self.uvw_bound, high=self.uvw_bound, size=(3,1))
        pqr = np.random.uniform(low=-self.pqr_bound, high=self.pqr_bound, size=(3,1))
        xyz[2,:] = abs(xyz[2,:])
        return xyz, zeta, uvw, pqr

    def reward(self, state, action):
        xyz, zeta, _, pqr = state

        #s_zeta = np.sin(zeta)
        #c_zeta = np.cos(zeta)

        curr_dist = xyz-self.goal
        #curr_att_sin = s_zeta-self.goal_zeta_sin
        #curr_att_cos = c_zeta-self.goal_zeta_cos
        #curr_ang = pqr-self.goal_pqr

        dist_hat = np.linalg.norm(curr_dist)
        #att_hat_sin = np.linalg.norm(curr_att_sin)
        #att_hat_cos = np.linalg.norm(curr_att_cos)
        #ang_hat = np.linalg.norm(curr_ang)

        # agent gets a negative reward based on how far away it is from the desired goal state
        if dist_hat > self.goal_thresh:
            dist_rew = 1/dist_hat
        else:
            dist_rew = 1/self.goal_thresh
        att_rew = 0#10*((self.att_norm_sin-att_hat_sin)+(self.att_norm_cos-att_hat_cos))
        ang_rew = 0#*(self.ang_norm-ang_hat)
        if dist_hat < 0.05:
            dist_rew += 50

        self.dist_norm = dist_hat
        #self.att_norm_sin = att_hat_sin
        #self.att_norm_cos = att_hat_cos
        #self.ang_norm = ang_hat

        self.vec_xyz = curr_dist
        #self.vec_zeta_sin = curr_att_sin
        #self.vec_zeta_cos = curr_att_cos
        #elf.vec_pqr = curr_ang

        ctrl_rew = 0#-np.sum(((action/self.action_bound[1])**2))
        time_rew = 1.
        return dist_rew, att_rew, ang_rew, ctrl_rew, time_rew

    def terminal(self, pos):
        xyz, zeta = pos
        mask1 = 0#zeta[0:2] > pi/2
        mask2 = 0#zeta[0:2] < -pi/2
        mask3 = self.dist_norm > 5
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
        tmp = zeta.T.tolist()[0]
        sinx = [sin(x) for x in tmp]
        cosx = [cos(x) for x in tmp]
        next_state = xyz.T.tolist()[0]+sinx+cosx+uvw.T.tolist()[0]+pqr.T.tolist()[0]+(action/self.action_bound[1]).tolist()
        info = self.reward((xyz, zeta, uvw, pqr), action)
        done = self.terminal((xyz, zeta))
        reward = sum(info)
        goal = self.vec.T.tolist()[0]
        self.t += self.ctrl_dt
        next_state = next_state+goal
        return next_state, reward, done, info

    def reset(self):
        self.goal_achieved = False
        self.t = 0.
        if self.deterministic_s0:
            xyz, zeta, uvw, pqr = self.iris.reset()
        else:
            xyz, zeta, uvw, pqr = self.generate_s0()
            self.iris.set_state(xyz, zeta, uvw, pqr)
        self.iris.set_rpm(np.array(self.trim))
        self.goal = self.generate_goal(self.r)
        self.vec = xyz-self.goal
        tmp = zeta.T.tolist()[0]
        action = [x/self.action_bound[1] for x in self.trim]
        sinx = [sin(x) for x in tmp]
        cosx = [cos(x) for x in tmp]
        state = xyz.T.tolist()[0]+sinx+cosx+uvw.T.tolist()[0]+pqr.T.tolist()[0]+action+self.vec.T.tolist()[0]
        return state

    def generate_goal(self, r):
        phi = random.uniform(-2*pi, 2*pi)
        theta = random.uniform(-2*pi, 2*pi)
        x = r*sin(theta)*cos(phi)
        y = r*sin(theta)*sin(phi)
        z = r*cos(theta)
        return np.array([[x],
                        [y],
                        [z]])

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
        self.vis.draw_goal(self.axis3d, self.goal)
        self.axis3d.set_xlim(-3, 3)
        self.axis3d.set_ylim(-3, 3)
        self.axis3d.set_zlim(-3, 3)
        self.axis3d.set_xlabel('West/East [m]')
        self.axis3d.set_ylabel('South/North [m]')
        self.axis3d.set_zlabel('Down/Up [m]')
        self.axis3d.set_title("Time %.3f s" %(self.t))
        pl.pause(0.001)
        pl.draw()
