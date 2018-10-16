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
from scipy.special import gammainc
import simulation.animation_gl as ani_gl

class TrajectoryEnv(gym.Env):
    """
    Environment wrapper for training low-level flying skills. In this environment, the aircraft
    has a deterministic starting state by default. We can switch it to have non-deterministic 
    initial states. This is obviously much harder.

    -- Sean Morrison
    """

    def __init__(self):
        metadata = {'render.modes': ['human']}
        self.r_max = 1.5
        self.goal_thresh = 0.075
        self.t = 0
        self.T = 6
        self.action_space = np.zeros((4,))
        self.observation_space = np.zeros((37,))

        # build list of waypoints for the aircraft to fly to
        self.traj_len = 2
        self.goal_list = []
        x = np.array([[0.],[0.],[0.]])
        for i in range(self.traj_len):
            x += self.generate_goal()
            self.goal_list.append(x.copy())
        print("Trajectory Length: ", len(self.goal_list))
        print("Waypoints: ")
        for g in self.goal_list:
            print(g.reshape(1,-1)[0])
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
        self.goal_pqr = np.array([[0.],
                                [0.],
                                [0.]])
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
        self.prev_action = self.trim_np.copy()
        self.bandwidth = 35.
        xyz, zeta, uvw, pqr = self.iris.get_state()
        self.vec_xyz = xyz-self.goal_xyz
        self.vec_zeta_sin = np.sin(zeta)-self.goal_zeta_sin
        self.vec_zeta_cos = np.cos(zeta)-self.goal_zeta_cos
        self.vec_pqr = pqr-self.goal_pqr
        self.dist_norm = np.linalg.norm(self.vec_xyz)
        self.att_norm_sin = np.linalg.norm(self.vec_zeta_sin)
        self.att_norm_cos = np.linalg.norm(self.vec_zeta_cos)
        self.ang_norm = np.linalg.norm(self.vec_pqr)
        self.init_rendering = False
        self.lazy_action = False
        self.lazy_change = False

    def set_lazy_action(self, lazy):
        """
        Parameters
        ----------
        lazy :

        Returns
        -------
            n/a
        """

        if lazy:
            self.lazy_action = True

    def set_lazy_change(self, lazy):
        """
        Parameters
        ----------
        lazy :

        Returns
        -------
            n/a
        """

        if lazy:
            self.lazy_change = True

    def get_goal(self, return_list=True):
        """
        Parameters
        ----------
        n/a

        Returns
        -------
            goal_xyz (numpy array):
                a 3x1 numpy array of the aircraft's goal position in Euclidean coordinates
        """

        if return_list:
            return self.goal_list
        else:
            return self.goal_xyz

    def get_next_goal(self):
        """
        Parameters
        ----------
        n/a

        Returns
        -------
            goal_xyz (numpy array):
                a 3x1 numpy array of the aircraft's goal position in Euclidean coordinates
        """

        return self.goal_xyz_next

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
        curr_dist = xyz-self.goal_xyz+self.datum
        curr_att_sin = s_zeta-self.goal_zeta_sin
        curr_att_cos = c_zeta-self.goal_zeta_cos
        curr_ang = pqr-self.goal_pqr
        
        # magnitude of the distance from the goal
        dist_hat = np.linalg.norm(curr_dist)
        att_hat_sin = np.linalg.norm(curr_att_sin)
        att_hat_cos = np.linalg.norm(curr_att_cos)
        ang_hat = np.linalg.norm(curr_ang)

        # agent gets a negative reward based on how far away it is from the desired goal state
        dist_rew = 100*(self.dist_norm-dist_hat)
        att_rew = 10*((self.att_norm_sin-att_hat_sin)+(self.att_norm_cos-att_hat_cos))
        ang_rew = 10*(self.ang_norm-ang_hat)

        self.dist_norm = dist_hat
        self.att_norm_sin = att_hat_sin
        self.att_norm_cos = att_hat_cos
        self.ang_norm = ang_hat

        self.vec_xyz = curr_dist
        self.vec_zeta_sin = curr_att_sin
        self.vec_zeta_cos = curr_att_cos
        self.vec_pqr = curr_ang

        if self.dist_norm <= self.goal_thresh:
            cmplt_rew = 100.
            self.goal_achieved(xyz)
            curr_dist = xyz-self.goal_xyz+self.datum
            dist_hat = np.linalg.norm(curr_dist)
            self.dist_norm = dist_hat
        else:
            cmplt_rew = 0

        # agent gets a negative reward for excessive action inputs
        ctrl_rew = 0.
        if self.lazy_action:
            ctrl_rew -= np.sum(((action-self.trim_np)/self.action_bound[1])**2)
        if self.lazy_change:
            ctrl_rew -= np.sum((((action-self.prev_action)/self.action_bound[1])**2))
        self.prev_action = action.copy()

        # agent gets a slight negative reward for time spent in flight
        time_rew = -0.1
        return dist_rew, att_rew, ang_rew, ctrl_rew, time_rew, cmplt_rew

    def terminal(self, pos):
        xyz, zeta = pos
        mask3 = self.dist_norm > 5.
        if np.sum(mask3) > 0:
            return True
        elif (self.dist_norm <= self.goal_thresh) and (self.goal_curr == self.traj_len-1):
            print("Last goal achieved!")
            return True
        elif self.t*self.ctrl_dt >= self.T-self.ctrl_dt:
            print("Sim time reached: {:.2f}s".format(self.t*self.ctrl_dt))
            return True
        else:
            return False
    
    def goal_achieved(self, xyz):
        print("Goal {} achieved".format(self.goal_curr))
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
        self.t += 1
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
        goal = position_goal+attitude_goal
        next_position_goal = self.goal_xyz_next.T.tolist()[0] 
        next_attitude_goal = self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]
        next_goal = next_position_goal+next_attitude_goal
        next_state = next_state+current_rpm+goal+next_goal
        return next_state, reward, done, {"dist_rew": info[0], 
                                        "att_rew": info[1], 
                                        "ctrl_rew": info[2], 
                                        "time_rew": info[3], 
                                        "cmplt_rew": info[4]}

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
        self.goal_list = []
        x = np.array([[0.],[0.],[0.]])
        for i in range(self.traj_len):
            x += self.generate_goal()
            self.goal_list.append(x.copy())
        self.goal_curr = 0
        self.goal_next_curr = self.goal_curr+1
        self.goal_xyz = self.goal_list[self.goal_curr]
        self.goal_xyz_next = self.goal_list[self.goal_next_curr]
        sin_zeta = np.sin(zeta)
        cos_zeta = np.cos(zeta)
        current_rpm = (self.iris.get_rpm()/self.action_bound[1]).tolist()
        next_position = xyz.T.tolist()[0]
        next_attitude = sin_zeta.T.tolist()[0]+cos_zeta.T.tolist()[0]
        next_velocity = uvw.T.tolist()[0]+pqr.T.tolist()[0]   
        next_state = next_position+next_attitude+next_velocity
        self.vec_xyz = xyz-self.goal_xyz
        self.vec_zeta_sin = sin_zeta-self.goal_zeta_sin
        self.vec_zeta_cos = cos_zeta-self.goal_zeta_cos
        self.vec_pqr = pqr - self.goal_pqr
        position_goal = self.vec_xyz.T.tolist()[0] 
        attitude_goal = self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]
        goal = position_goal+attitude_goal
        next_position_goal = self.goal_xyz_next.T.tolist()[0] 
        next_attitude_goal = self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]
        next_goal = next_position_goal+next_attitude_goal
        next_state = next_state+current_rpm+goal+next_goal
        return next_state

    def generate_goal(self):
        """
        Parameters
        ----------
        n/a

        Returns
        -------
            goal_xyz (numpy array):
                a 3x1 numpy array of the aircraft's goal position in Euclidean coordinates
        """

        def sample(center,radius,n_per_sphere):
            """
            Parameters
            ----------
            center :
            radius :
            n_per_sphere :

            Returns
            -------
                p (numpy array) :
                    a size (3,) numpy array of the aircraft's goal position in Euclidean coordinates,
                    sampled uniformly from the volume of a sphere of given radius. 
            """

            r = radius
            ndim = center.size
            x = np.random.normal(size=(n_per_sphere, ndim))
            ssq = np.sum(x**2,axis=1)
            fr = r*gammainc(ndim/2,ssq/2)**(1/ndim)/np.sqrt(ssq)
            frtiled = np.tile(fr.reshape(n_per_sphere,1),(1,ndim))
            p = center + np.multiply(x,frtiled)
            return p
        goal_xyz = sample(np.array([0.,0.,0.]), self.r_max, 1).reshape(-1,1) 
        return goal_xyz
    
    def render(self, mode='human', close=False):
        if not self.init_rendering:
            self.ani = ani_gl.VisualizationGL(name="Trajectory")
            self.init_rendering = True
        self.ani.draw_quadrotor(self.iris)
        for g in self.goal_list:
            self.ani.draw_goal(g)
        self.ani.draw_label("Time: {0:.2f}".format(self.t*self.ctrl_dt), 
            (self.ani.window.width // 2, 20.0))
        self.ani.draw()


