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
        has a deterministic starting state, and we generate a random waypoint for it to navigate to.
        This is similar to the static waypoint task, but much harder. The observation space of the
        quadrotor is important here, because it needs to be able to see the goal state.
    """
    def __init__(self):
        metadata = {'render.modes': ['human']}
        self.r_max = 2.5
        self.goal_thresh = 0.05
        self.t = 0
        self.T = self.generate_time()
        self.action_space = np.zeros((4,))
        self.observation_space = np.zeros((34,))

        self.goal_xyz = self.generate_goal()
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
        self.points = self.generate_guide(xyz)
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

        self.fig = None
        self.axis3d = None

    def reward(self, state, action):
        xyz, zeta, uvw, pqr = state
        #s_zeta = np.sin(zeta)
        #c_zeta = np.cos(zeta)
        curr_dist = xyz-self.goal_xyz
        curr_traj = xyz-self.points[self.t]
        #curr_att_sin = s_zeta-self.goal_zeta_sin
        #curr_att_cos = c_zeta-self.goal_zeta_cos
        #curr_vel = uvw-self.goal_uvw
        #curr_ang = pqr-self.goal_pqr
        
        # magnitude of the distance from the goal 
        dist_hat = np.linalg.norm(curr_dist)
        traj_hat = np.linalg.norm(curr_dist)
        #att_hat_sin = np.linalg.norm(curr_att_sin)
        #att_hat_cos = np.linalg.norm(curr_att_cos)
        #vel_hat = np.linalg.norm(curr_vel)
        #ang_hat = np.linalg.norm(curr_ang)

        # agent gets a negative reward based on how far away it is from the desired goal state
        dist_rew = 100*(self.dist_norm-dist_hat)
        #att_rew = 10*((self.att_norm_sin-att_hat_sin)+(self.att_norm_cos-att_hat_cos))
        #vel_rew = 0.1*(self.vel_norm-vel_hat)
        #ang_rew = 0.1*(self.ang_norm-ang_hat)
        #self.dist_norm = dist_hat
        #self.att_norm_sin = att_hat_sin
        #self.att_norm_cos = att_hat_cos
        #self.vel_norm = vel_hat
        #self.ang_norm = ang_hat
        #self.vec_xyz = curr_dist
        #self.vec_zeta_sin = curr_att_sin
        #self.vec_zeta_cos = curr_att_cos
        #self.vec_uvw = curr_vel
        #self.vec_pqr = curr_ang
        
        if self.dist_norm <= self.goal_thresh:
            cmplt_rew = 100.
        else:
            cmplt_rew = 0
        
        # agent gets a negative reward for excessive action inputs
        ctrl_rew = -np.sum(((action/self.action_bound[1])**2))
        
        # agent gets a positive reward for time spent in flight
        time_rew = 0.1
        
        return dist_rew, att_rew, vel_rew, ang_rew, ctrl_rew, time_rew, cmplt_rew

    def terminal(self, pos):
        xyz, zeta = pos
        mask1 = 0#zeta[0:2] > pi/2
        mask2 = 0#zeta[0:2] < -pi/2
        mask3 = self.dist_norm > 5
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
        a = (action/self.action_bound[1]).tolist()
        next_state = xyz.T.tolist()[0]+sin_zeta.T.tolist()[0]+cos_zeta.T.tolist()[0]+uvw.T.tolist()[0]+pqr.T.tolist()[0]
        info = self.reward((xyz, zeta, uvw, pqr), action)
        done = self.terminal((xyz, zeta))
        reward = sum(info)
        goals = self.vec_xyz.T.tolist()[0]+self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]+self.vec_uvw.T.tolist()[0]+self.vec_pqr.T.tolist()[0]
        next_state = next_state+a+goals
        self.t += 1
        return next_state, reward, done, info

    def reset(self):
        self.t = 0.
        self.T = self.generate_time()
        self.H = int(self.T/self.ctrl_dt)
        xyz, zeta, uvw, pqr = self.iris.reset()
        self.points = self.generate_guide(xyz)
        self.iris.set_rpm(np.array(self.trim))
        self.goal_xyz = self.generate_goal()
        sin_zeta = np.sin(zeta)
        cos_zeta = np.cos(zeta)
        self.vec_xyz = xyz-self.goal_xyz
        self.vec_zeta_sin = sin_zeta
        self.vec_zeta_cos = cos_zeta
        self.vec_uvw = uvw
        self.vec_pqr = pqr
        a = (self.trim_np/self.action_bound[1]).tolist()
        goals = self.vec_xyz.T.tolist()[0]+self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]+self.vec_uvw.T.tolist()[0]+self.vec_pqr.T.tolist()[0]
        state = xyz.T.tolist()[0]+sin_zeta.T.tolist()[0]+cos_zeta.T.tolist()[0]+uvw.T.tolist()[0]+pqr.T.tolist()[0]+a+goals
        return state

    def generate_guide(self, xyz):
        pts = np.arange(xyz, self.goal_xyz, self.H)
        return pts[:,1:]

    def generate_goal(self):
        rad = np.random.uniform(low=1, high=self.r_max)
        x = np.random.uniform(low=-1, high=1, size=(3,1))
        x_hat = x/np.linalg.norm(x)
        xyz = rad*x_hat
        return xyz
    
    def generate_time(self):
        return np.random.uniform(low=1.5, high=5)
    
    def get_goal(self):
        return self.goal_xyz
    
    def get_time(self):
        return self.T
    
    def render(self, mode='human', close=False):
        if self.fig is None:
            # rendering parameters
            pl.close("all")
            pl.ion()
            self.fig = pl.figure("Random Waypoint")
            self.axis3d = self.fig.add_subplot(111, projection='3d')
            self.vis = ani.Visualization(self.iris, 6, quaternion=True)            
        pl.figure("Random Waypoint")
        self.axis3d.cla()
        self.vis.draw3d_quat(self.axis3d)
        self.vis.draw_goal(self.axis3d, self.goal_xyz)
        self.axis3d.set_xlim(-3, 3)
        self.axis3d.set_ylim(-3, 3)
        self.axis3d.set_zlim(-3, 3)
        self.axis3d.set_xlabel('West/East [m]')
        self.axis3d.set_ylabel('South/North [m]')
        self.axis3d.set_zlabel('Down/Up [m]')
        self.axis3d.set_title("Time %.3f s" %(self.t*self.ctrl_dt))
        pl.pause(0.001)
        pl.draw()
