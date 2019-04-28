import simulation.quadrotor3 as quad
import simulation.config as cfg
import numpy as np
import random
from math import pi, sin, cos, acos
import gym
from gym import error, spaces, utils
from gym.utils import seeding
import simulation.animation_gl as ani_gl

class TrajectoryEnv(gym.Env):
    """
    Environment wrapper for training low-level flying skills. The aim is to sequentially fly to
    two consecutive waypoints that are each uniformly sampled from the volume of a sphere. The
    first sphere is centered on the starting point (0,0,0), and the second sphere is centered on
    the point (xg,yg,zg). The agent is able to see both waypoints.
    
    The aircraft has a deterministic starting state by default.

    -- Sean Morrison
    """

    def __init__(self):
        metadata = {'render.modes': ['human']}
        self.r_max = 1.
        self.goal_thresh = 0.05
        self.t = 0
        self.T = 3
        
        # build list of waypoints for the aircraft to fly to
        self.traj_len = 4
        self.goal_list = []
        self.zeta_list = []
        x = np.zeros((3,1))
        for _ in range(self.traj_len):
            new_waypoint = x+self.generate_goal()
            self.goal_list.append(new_waypoint)
            sin_psi, cos_psi = self.generate_yaw(x, new_waypoint)
            self.zeta_list.append([np.array([[0.],[0.],[sin_psi],]), np.array([[1.],[1.],[cos_psi]])])
            x = new_waypoint
        self.goal = 0
        self.goal_next = self.goal+1

        self.goal_xyz = self.goal_list[self.goal]
        self.goal_xyz_next = self.goal_list[self.goal_next]
        self.goal_zeta_sin = self.zeta_list[self.goal][0]
        self.goal_zeta_cos = self.zeta_list[self.goal][1]
        self.goal_zeta_sin_next = self.zeta_list[self.goal_next][0]
        self.goal_zeta_cos_next = self.zeta_list[self.goal_next][1]
        self.goal_uvw = np.zeros((3,1))
        self.goal_pqr = np.zeros((3,1))
        self.datum = np.zeros((3,1))

        # the velocity of the aircraft in the inertial frame is probably a better metric here, but
        # since our goal state is (0,0,0), this should be fine.
        self.goal_uvw = np.zeros((3,1))
        self.goal_pqr = np.zeros((3,1))
        self.datum = np.zeros((3,1))

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
        self.action_space = spaces.Box(low=0, high=self.iris.max_rpm, shape=(4,))
        self.observation_space = np.zeros((50,))
        self.bandwidth = 35.
        
        self.init_rendering = False

    def sample(self):
        """
        Parameters
        ----------
        n/a

        Returns
        -------
            action (numpy array):
                a shape (4,) numpy array of random actions sampled from the action space
        """

        action = self.action_bound[1]*np.random(self.action_space.shape)
        return action

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
        curr_vel = uvw-self.goal_uvw
        curr_ang = pqr-self.goal_pqr

        # magnitude of the distance from the goal
        dist_hat = np.linalg.norm(curr_dist)
        att_hat_sin = np.linalg.norm(curr_att_sin)
        att_hat_cos = np.linalg.norm(curr_att_cos)
        vel_hat = np.linalg.norm(curr_vel)
        ang_hat = np.linalg.norm(curr_ang)
        
        guide_rew = 0.
        #if dist_hat <= self.goal_thresh:
        #    guide_rew = 1./self.goal_thresh**2
        #else:
        #    guide_rew = 1./dist_hat**2
        
        dist_rew = 100.*(self.dist_norm-dist_hat)
        att_rew = 100*((self.att_norm_sin-att_hat_sin)+(self.att_norm_cos-att_hat_cos))
        vel_rew = 50.*(self.vel_norm-vel_hat)
        ang_rew = 50.*(self.ang_norm-ang_hat)

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
        ctrl_rew -= 50.*np.sum((uvw-self.prev_uvw)**2)
        ctrl_rew -= 50.*np.sum((pqr-self.prev_pqr)**2)

        # agent gets a slight negative reward for time spent in flight
        time_rew = 0.
        return dist_rew, att_rew, vel_rew, ang_rew, ctrl_rew, time_rew, guide_rew

    def terminal(self):
        mask3 = self.dist_norm > 5.
        if np.sum(mask3) > 0:
            return True
        #elif (self.dist_norm <= self.goal_thresh) and (self.goal_curr == self.traj_len-1):
            #print("Last goal achieved!")
            #return True
        elif self.t*self.ctrl_dt >= self.T:
            #print("Sim time reached: {:.2f}s".format(self.t*self.ctrl_dt))
            return True
        else:
            return False
    
    def next_goal(self, state):
        xyz, zeta, uvw, pqr = state
        if not self.goal >= len(self.goal_list)-1:
            self.time_state = float(self.T)
            self.t = 0
            self.datum = xyz.copy()
            self.goal += 1
            self.goal_xyz = self.goal_list[self.goal]
            self.goal_zeta_sin = self.zeta_list[self.goal][0]
            self.goal_zeta_cos = self.zeta_list[self.goal][1]
        if self.goal_next >= len(self.goal_list)-1:
            self.goal_xyz_next = np.zeros((3,1))
            self.goal_zeta_sin_next = np.zeros((3,1))
            self.goal_zeta_cos_next = np.zeros((3,1))
        else:
            self.goal_next += 1
            self.goal_xyz_next = self.goal_list[self.goal_next]
            self.goal_zeta_sin_next = self.zeta_list[self.goal_next][0]
            self.goal_zeta_cos_next = self.zeta_list[self.goal_next][1]
        
        s_zeta = np.sin(zeta)
        c_zeta = np.cos(zeta)
        curr_dist = xyz-self.goal_xyz+self.datum
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

        # save new magnitudes
        self.dist_norm = dist_hat
        self.att_norm_sin = att_hat_sin
        self.att_norm_cos = att_hat_cos
        self.vel_norm = vel_hat
        self.ang_norm = ang_hat

        # save new vectors
        self.vec_xyz = curr_dist
        self.vec_zeta_sin = curr_att_sin
        self.vec_zeta_cos = curr_att_cos
        self.vec_uvw = curr_vel
        self.vec_pqr = curr_ang

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

        rpm_command = self.trim_np+action*self.bandwidth
        for _ in self.steps:
            xs, zeta, uvw, pqr = self.iris.step(rpm_command)
        self.time_state -= self.ctrl_dt
        xyz = xs.copy()-self.datum
        sin_zeta = np.sin(zeta)
        cos_zeta = np.cos(zeta)
        current_rpm = (self.iris.get_rpm()/self.action_bound[1]).tolist()
        next_position = xyz.T.tolist()[0]
        next_attitude = sin_zeta.T.tolist()[0]+cos_zeta.T.tolist()[0]
        next_velocity = uvw.T.tolist()[0]+pqr.T.tolist()[0]
        next_state = next_position+next_attitude+next_velocity
        info = self.reward((xyz, zeta, uvw, pqr), self.trim_np+action*self.bandwidth)
        
        if self.dist_norm <= self.goal_thresh:
            self.next_goal((xs, zeta, uvw, pqr))
        
        done = self.terminal()
        reward = sum(info)
        position_goal = self.vec_xyz.T.tolist()[0]
        attitude_goal = self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]
        velocity_goal = self.vec_uvw.T.tolist()[0]+self.vec_pqr.T.tolist()[0]
        goals = position_goal+attitude_goal+velocity_goal
        next_position_goal = (xyz-self.goal_xyz_next).T.tolist()[0]
        next_attitude_goal = (sin_zeta-self.goal_zeta_sin_next).T.tolist()[0]+(cos_zeta-self.goal_zeta_cos_next).T.tolist()[0]
        next_velocity_goal = self.vec_uvw.T.tolist()[0]+self.vec_pqr.T.tolist()[0]
        next_goals = next_position_goal+next_attitude_goal+next_velocity_goal
        next_state = next_state+current_rpm+goals+next_goals+[self.time_state]
        self.prev_action = rpm_command
        self.prev_uvw = uvw
        self.prev_pqr = pqr
        self.t += 1
        return next_state, reward, done, {"dist_rew": info[0], 
                                        "dir_rew": info[1], 
                                        "vel_rew": info[2], 
                                        "ang_rew": info[3],
                                        "ctrl_rew": info[4],
                                        "time_rew": info[5],
                                        }

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

        # reset env state, aircraft simulation
        self.t = 0
        self.time_state = float(self.T)
        xyz, zeta, uvw, pqr = self.iris.reset()
        self.prev_action = self.trim_np
        self.prev_uvw = np.zeros((3,1))
        self.prev_pqr = np.zeros((3,1))
        self.datum = np.zeros((3,1))

        # generate new set of trajectory waypoints
        self.goal_list = []
        self.zeta_list = []
        x = np.zeros((3,1))
        for _ in range(self.traj_len):
            new_waypoint = x+self.generate_goal()
            self.goal_list.append(new_waypoint)
            sin_psi, cos_psi = self.generate_yaw(x, new_waypoint)
            self.zeta_list.append([np.array([[0.],[0.],[sin_psi],]), np.array([[1.],[1.],[cos_psi]])])
            x = new_waypoint
        self.goal = 0
        self.goal_next = self.goal+1

        # set next goal, and subsequent goal
        self.goal_xyz = self.goal_list[self.goal]
        self.goal_xyz_next = self.goal_list[self.goal_next]
        self.goal_zeta_sin = self.zeta_list[self.goal][0]
        self.goal_zeta_cos = self.zeta_list[self.goal][1]
        self.goal_zeta_sin_next = self.zeta_list[self.goal_next][0]
        self.goal_zeta_cos_next = self.zeta_list[self.goal_next][1]

        # reset state observation
        s_zeta = np.sin(zeta)
        c_zeta = np.cos(zeta)
        current_rpm = (self.iris.get_rpm()/self.action_bound[1]).tolist()
        position = xyz.T.tolist()[0]
        attitude = s_zeta.T.tolist()[0]+c_zeta.T.tolist()[0]
        velocity = uvw.T.tolist()[0]+pqr.T.tolist()[0]   
        state = position+attitude+velocity
        
        # calculate vector dist to current goal
        curr_dist = xyz-self.goal_xyz+self.datum
        curr_att_sin = s_zeta-self.goal_zeta_sin
        curr_att_cos = c_zeta-self.goal_zeta_cos
        curr_vel = uvw-self.goal_uvw
        curr_ang = pqr-self.goal_pqr

        # calculate magnitude of dist vectors
        dist_hat = np.linalg.norm(curr_dist)
        att_hat_sin = np.linalg.norm(curr_att_sin)
        att_hat_cos = np.linalg.norm(curr_att_cos)
        vel_hat = np.linalg.norm(curr_vel)
        ang_hat = np.linalg.norm(curr_ang)

        # save magnitude of dist vectors (since we are rewarding based on change in distance)
        self.dist_norm = dist_hat
        self.att_norm_sin = att_hat_sin
        self.att_norm_cos = att_hat_cos
        self.vel_norm = vel_hat
        self.ang_norm = ang_hat

        # save current vectors
        self.vec_xyz = curr_dist
        self.vec_zeta_sin = curr_att_sin
        self.vec_zeta_cos = curr_att_cos
        self.vec_uvw = curr_vel
        self.vec_pqr = curr_ang
        
        # add immediate goal to state observation
        position_goal = self.vec_xyz.T.tolist()[0]
        attitude_goal = self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]
        velocity_goal = self.vec_uvw.T.tolist()[0]+self.vec_pqr.T.tolist()[0]
        goals = position_goal+attitude_goal+velocity_goal

        # add subsequent goal to state observation
        next_position_goal = (xyz-self.goal_xyz_next).T.tolist()[0]
        next_attitude_goal = (s_zeta-self.goal_zeta_sin_next).T.tolist()[0]+(c_zeta-self.goal_zeta_cos_next).T.tolist()[0]
        next_velocity_goal = self.vec_uvw.T.tolist()[0]+self.vec_pqr.T.tolist()[0]
        next_goals = next_position_goal+next_attitude_goal+next_velocity_goal
        
        # return full state observation
        state = state+current_rpm+goals+next_goals+[self.time_state]      
        return state

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
       
        # do rejection sampling to draw uniformly from a hemisphere forward of the aircraft
        x = np.random.uniform(low=0, high=self.r_max)
        y = np.random.uniform(low=-self.r_max/2., high=self.r_max/2.)
        z = np.random.uniform(low=-self.r_max/2., high=self.r_max/2.)
        goal_xyz = np.array([[x],[y],[z]])
        #mag = np.linalg.norm(goal_xyz)
        return goal_xyz#(goal_xyz/mag)*self.r_max
    
    def generate_yaw(self, v1, v2):
        direction = v2-v1
        xy = direction[:-1,:]
        mag = np.linalg.norm(xy)
        sin_psi, cos_psi = xy[1,:]/mag, xy[0,:]/mag
        return sin_psi, cos_psi
    
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
        self.ani.draw_goal(np.zeros((3,1)))
        for i, g in enumerate(self.goal_list):
            if i == 0:
                self.ani.draw_line(np.zeros((3,1)), g)
            else:
                self.ani.draw_line(self.goal_list[i-1], g)
            if g is self.goal_xyz:
                c = (0.5, 0., 0.)
            else:
                c = (0., 0.5, 0.)
            self.ani.draw_goal(g, color=c)
        self.ani.draw_label("Time: {0:.2f}".format(self.t*self.ctrl_dt),
            (self.ani.window.width // 2, 20.0))
        self.ani.draw()
    
    def record(self):
        self.ani.record()