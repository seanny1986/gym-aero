import simulation.quadrotor3 as quad
import simulation.config as cfg
import simulation.animation as ani
import matplotlib.pyplot as pl
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
import random
from math import pi, sin, cos
import math
import gym
from gym import error, spaces, utils
from gym.utils import seeding
from gym_aero.envs.box_world_helper import Sphere
import numpy as np
import simulation.animation_gl as ani_gl

class BoxWorld(gym.Env):
    def __init__(self, num_obstacles=6, max_rad=2, length=5, width=5, height=5):
        metadata = {'render.modes': ['human']}
        self.num_obstacles = num_obstacles
        self.max_rad = max_rad
        self.length = length
        self.width = width
        self.height = height
        self.goal_thresh = 0.05
        self.t = 0
        self.T = 10
        self.action_space = np.zeros((4,))
        self.observation_space = np.zeros((34+int(num_obstacles*3),))

        # final goal
        self.goal_xyz = self.generate_goal()


        # waypoint goals
        self.wp_curr_xyz = None
        self.wp_next_xyz = None
        self.wp_zeta_sin = np.sin(np.array([[0.],
                                            [0.],
                                            [0.]]))
        self.wp_zeta_cos = np.cos(np.array([[0.],
                                            [0.],
                                            [0.]]))
        self.wp_uvw = np.array([[0.],
                                [0.],
                                [0.]])
        self.wp_pqr = np.array([[0.],
                                [0.],
                                [0.]])
        self.datum = np.zeros((3,1))
        self.traj_len = 3
        self.waypoint_list = []
        self.waypoints_reached = 0

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

        self.spawn= xyz
        self.vec_xyz = xyz-self.wp_curr_xyz
        self.vec_zeta_sin = np.sin(zeta)-self.wp_zeta_sin
        self.vec_zeta_cos = np.cos(zeta)-self.wp_zeta_cos
        self.vec_uvw = uvw-self.wp_uvw
        self.vec_pqr = pqr-self.wp_pqr

        self.dist_norm = np.linalg.norm(self.vec_xyz)
        self.att_norm_sin = np.linalg.norm(self.vec_zeta_sin)
        self.att_norm_cos = np.linalg.norm(self.vec_zeta_cos)
        self.vel_norm = np.linalg.norm(self.vec_uvw)
        self.ang_norm = np.linalg.norm(self.vec_pqr)

        # generate collision points
        n = 10
        self.col_rad = self.iris.l+self.iris.get_prop_radius()
        xs = [self.col_rad*cos(2*pi*(i/n)) for i in range(n)]
        ys = [self.col_rad*sin(2*pi*(i/n)) for i in range(n)]
        self.pts = [np.array([[0.], [xs[i]],[ys[i]],[0.]]) for i in range(n)]

        self.obstacles = self.generate_obstacles()

        # necessary functions
        self.q_mult = self.iris.q_mult
        self.q_conj = self.iris.q_conj

        # for open_gl animation
        self.init_rendering = False
    
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

    def generate_obstacles(self):
        # generate obstacles
        xyz = np.zeros(3,1)
        obstacles = []
        for i in range(self.num_obstacles):
            collision = True
            while collision:
                obs = Sphere(self.max_rad, self.length, self.width, self.height)
                collision = np.linalg.norm(xyz-obs.xyz)<= self.col_rad+obs.rad
            obstacles.append(obs)
        return obstacles
    
    def generate_goal(self):
        def gen_coords():
            x = random.uniform(-self.length/2, self.length/2)
            y = random.uniform(-self.width/2, self.width/2)
            z = random.uniform(-self.height/2, self.height/2)
            return  np.array([[x],[y],[z]])
        collision = True
        while collision:
            goal_xyz = gen_coords()
        return goal_xyz

    def get_obstacles(self):
        return self.obstacles

    def waypoint_achieved(self, xyz):
        self.waypoints_reached += 1
        self.datum = xyz.copy()
        self.pop_waypoint()
        self.process_waypoints()
    
    def process_waypoints(self):
        self.wp_curr_xyz = self.waypoint_list[0]
        self.wp_next_xyz = self.waypoint_list[1]

    #Checks if object is closer than the smallest circle
    #i.e if distance to object is less than any point on level 0 circle
    def check_collision(self, pos):
        quat = self.iris.get_q()
        Q = self.q_mult(quat)
        Q_inv = self.q_conj(quat)
        rotation = [Q.dot(self.q_mult(xyz).dot(Q_inv))[1:] for xyz in self.pts]
        translation = rotation+pos
        cols = [[np.linalg.norm(x-s.xyz)<= self.col_rad+s.rad for x in translation] for s in self.obstacles]
        return sum(cols) > 0

    def get_goal(self):
        return self.goal_xyz

    def add_waypoint(self, waypoint):
        self.waypoint_list.append(waypoint)
    
    def pop_waypoint(self):
        self.waypoint_list.remove(0)

    def policy_reward(self, state, action):
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
        curr_dist = xyz-self.wp_curr_xyz+self.datum
        curr_att_sin = s_zeta-self.wp_zeta_sin
        curr_att_cos = c_zeta-self.wp_zeta_cos
        curr_vel = uvw-self.wp_uvw
        curr_ang = pqr-self.wp_pqr

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
        ang_rew = 10*(self.ang_norm-ang_hat)
        self.dist_norm = dist_hat
        self.att_norm_sin = att_hat_sin
        self.att_norm_cos = att_hat_cos
        self.vel_norm = vel_hat
        self.ang_norm = ang_hat
        self.vec_xyz = curr_dist
        self.vec_zeta_sin = curr_att_sin
        self.vec_zeta_cos = curr_att_cos
        self.vec_pqr = curr_ang
        if self.dist_norm <= self.goal_thresh:
            cmplt_rew = (self.waypoints_reached+1)*100.
            self.waypoint_achieved(xyz)
            curr_dist = xyz-self.wp_curr_xyz+self.datum
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
            ctrl_rew -= np.sum((uvw-self.prev_uvw)**2)
            ctrl_rew -= np.sum((pqr-self.prev_pqr)**2)

        # agent gets a slight negative reward for time spent in flight
        time_rew = 0.
        return dist_rew, att_rew, vel_rew, ang_rew, ctrl_rew, time_rew, cmplt_rew

    def policy_terminal(self, pos):
        xyz, zeta ,uvw= pos
        mask1 = np.abs(xyz[0]) > self.length
        mask2 = np.abs(xyz[1]) > self.width
        mask3 = np.abs(xyz[2]) > self.height
        if mask1 or mask2 or mask3:
            return True
        elif self.check_collision(xyz):
            return True
        elif self.t*self.ctrl_dt >= self.T*(1+self.waypoints_reached):
            return True
        else:
            return False

    def planner_step(self, waypoint, action):
        next_state = waypoint+action
        d1 = np.linalg.norm(self.goal_xyz-waypoint)
        d2 = np.linalg.norm(self.goal_xyz-next_state)
        dist_rew = d1-d2
        col = self.check_collision(next_state)
        if col:
            col_rew = -10
            terminate = True
        else:
            col_rew = 10
            terminate = False
        rew = dist_rew+col_rew
        return next_state, rew, terminate, {"dist_rew": dist_rew,
                                            "col_rew": col_rew}

    def policy_step(self, action):
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
        rpm_command = self.trim_np+action*self.bandwidth
        for _ in self.steps:
            xs, zeta, uvw, pqr = self.iris.step(rpm_command)
        xyz = xs.copy()-self.datum
        sin_zeta = np.sin(zeta)
        cos_zeta = np.cos(zeta)
        current_rpm = (self.iris.get_rpm()/self.action_bound[1]).tolist()
        next_position = xyz.T.tolist()[0]
        next_attitude = sin_zeta.T.tolist()[0]+cos_zeta.T.tolist()[0]
        next_velocity = uvw.T.tolist()[0]+pqr.T.tolist()[0]
        next_state = next_position+next_attitude+next_velocity
        info = self.policy_reward((xyz, zeta, uvw, pqr), self.trim_np+action*self.bandwidth)
        done = self.policy_terminal((xyz, zeta))
        reward = sum(info)
        position_obs = sum([(xyz-obs.xyz).T.tolist()[0] for obs in self.obstacles],[])
        position_goal = self.vec_xyz.T.tolist()[0]
        attitude_goal = self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]
        goal = position_goal+attitude_goal
        next_position_goal = (xyz-self.wp_next_xyz).T.tolist()[0]
        next_attitude_goal = self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]
        next_goal = next_position_goal+next_attitude_goal
        next_state = next_state+current_rpm+position_obs+goal+next_goal
        self.prev_action = rpm_command.copy()
        self.prev_uvw = uvw.copy()
        self.prev_pqr = pqr.copy()
        self.t += 1
        return next_state, reward, done, {"dist_rew": info[0], 
                                        "att_rew": info[1], 
                                        "vel_rew": info[2], 
                                        "ang_rew": info[3],
                                        "ctrl_rew": info[4],
                                        "time_rew": info[5], 
                                        "cmplt_rew": info[6]}

    def planner_reset(self):
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
        xyz, _, _, _ = self.iris.get_state()
        state = xyz.T.tolist()[0]
        position_obs = [(xyz-obs.xyz).T.tolist()[0] for obs in self.obstacles]
        return state+position_obs

    def policy_reset(self):
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
        self.waypoints_reached = 0
        xyz, zeta, uvw, pqr = self.iris.reset()
        self.iris.set_rpm(np.array(self.trim))
        self.datum = np.zeros((3,1))
        self.waypoint_list = []
        sin_zeta = np.sin(zeta)
        cos_zeta = np.cos(zeta)
        current_rpm = (self.iris.get_rpm()/self.action_bound[1]).tolist()
        next_position = xyz.T.tolist()[0]
        next_attitude = sin_zeta.T.tolist()[0]+cos_zeta.T.tolist()[0]
        next_velocity = uvw.T.tolist()[0]+pqr.T.tolist()[0]   
        next_state = next_position+next_attitude+next_velocity
        self.vec_xyz = xyz-self.wp_curr_xyz
        self.vec_zeta_sin = sin_zeta-self.wp_zeta_sin
        self.vec_zeta_cos = cos_zeta-self.wp_zeta_cos
        self.vec_uvw = uvw-self.wp_uvw
        self.vec_pqr = pqr - self.wp_pqr
        self.dist_norm = np.linalg.norm(self.vec_xyz)
        self.att_norm_sin = np.linalg.norm(self.vec_zeta_sin)
        self.att_norm_cos = np.linalg.norm(self.vec_zeta_cos)
        self.vel_norm = np.linalg.norm(self.vec_uvw)
        self.ang_norm = np.linalg.norm(self.vec_pqr)
        position_obs = sum([(xyz-obs.xyz).T.tolist()[0] for obs in self.obstacles],[])
        position_goal = self.vec_xyz.T.tolist()[0]
        attitude_goal = self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]
        goal = position_goal+attitude_goal
        next_position_goal = (xyz-self.wp_next_xyz).T.tolist()[0]
        next_attitude_goal = self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]
        next_goal = next_position_goal+next_attitude_goal
        next_state = next_state+current_rpm+position_obs+goal+next_goal
        return next_state

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
            self.ani = ani_gl.VisualizationGL(name="Trajectory")
            self.init_rendering = True
        self.ani.draw_quadrotor(self.iris)
        for wp in self.waypoint_list:
            self.ani.draw_goal(wp,color=(0.5,0,0))
        for s in self.obstacles:
            self.ani.draw_sphere(s.get_center(), s.get_radius())
        self.ani.draw_goal(self.goal_xyz)
        self.ani.draw_label("Time: {0:.2f}".format(self.t*self.ctrl_dt), 
            (self.ani.window.width // 2, 20.0))
        self.ani.draw()
