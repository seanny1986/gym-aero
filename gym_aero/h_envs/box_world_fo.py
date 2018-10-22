import simulation.quadrotor3 as quad
import simulation.config as cfg
import numpy as np
import random
from math import pi, sin, cos
import gym
from gym import error, spaces, utils
from gym.utils import seeding
from gym_aero.h_envs.helper import sample
from gym_aero.h_envs.helper import Sphere
import numpy as np
import simulation.animation_gl as ani_gl

class BoxWorld(gym.Env):
    def __init__(self, num_obstacles=7, max_rad=1., length=5, width=5, height=5):
        print("----RUNNING ENVIRONMENT SETUP----")
        metadata = {'render.modes': ['human']}
        self.num_obstacles = num_obstacles
        self.max_rad = max_rad
        self.length = length
        self.width = width
        self.height = height
        self.max_step = 1.
        self.steps_wp = 25
        self.count = 0
        self.goal_thresh = 0.1
        self.t = 0
        self.T = 5
        self.count = 0
        self.action_space = np.zeros((4,))
        self.observation_space = np.zeros((37+int(num_obstacles*4),))
        self.planner_action_space = np.zeros((3,))
        self.planner_observation_space = np.zeros((3+3+int(num_obstacles*4),))
        print("Action and Observation spaces initialized")

        # waypoint goals
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
        self.zero = np.zeros((3,1))
        self.datum = np.zeros((3,1))
        self.waypoint_list = []
        self.waypoints_reached = 0

        # simulation parameters
        print("Initializing aircraft")
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
        self.prev_uvw = np.zeros((3,1))
        self.prev_pqr = np.zeros((3,1))
        self.bandwidth = 35.

        # necessary functions
        self.q_mult = self.iris.q_mult
        self.q_conj = self.iris.q_conj

        # generate collision points
        print("Generating collision points")
        n = 6
        self.col_rad = self.params["l"]+self.params["prop_radius"]
        xs = [self.col_rad*cos(2*pi*(i/n)) for i in range(n)]
        ys = [self.col_rad*sin(2*pi*(i/n)) for i in range(n)]
        self.pts = [np.array([[0.], [xs[i]],[ys[i]],[0.]]) for i in range(n)]

        print("Generating obstacles")
        self.obstacles = self.generate_obstacles()

        # final goal
        print("Generating goal")
        self.goal_xyz = self.generate_goal()

        # don't want to keep planning if we're at the goal
        self.wp_curr = 0
        self.wp_next = 1
        self.wp_xyz = None
        self.wp_xyz_next = None
        self.pol_col = False
        self.pla_col = False

        # for open_gl animation
        self.init_rendering = False

    def generate_obstacles(self):
        obstacles = []
        for i in range(self.num_obstacles):
            collision = True
            while collision:
                obs = Sphere(self.max_rad, 5)
                cols = [np.linalg.norm(p[1:]-obs.xyz)<= self.col_rad+obs.rad for p in self.pts]
                collision = sum(cols) > 0
            obstacles.append(obs)
        return obstacles
    
    def generate_goal(self):
        collision = True
        while collision:
            goal_xyz = sample(np.zeros(3,), 5, 1).reshape(-1,1)
            collision = self.check_collision(goal_xyz, 0.)
        return goal_xyz

    def init_waypoints(self):
        if self.waypoint_list:
            if len(self.waypoint_list) >= 2:
                self.wp_xyz = self.waypoint_list[self.wp_curr]
                self.wp_xyz_next = self.waypoint_list[self.wp_next]
            else:
                self.wp_xyz = self.waypoint_list[self.wp_curr]
                self.wp_xyz_next = np.zeros((3,1))
        else:
            print("Waypoint list is empty")

    def waypoint_achieved(self, xyz):
        self.waypoints_reached += 1
        if not self.wp_curr >= len(self.waypoint_list)-1:
            self.datum = xyz.copy()
            self.wp_curr += 1
            self.wp_xyz = self.waypoint_list[self.wp_curr]
        if self.wp_next >= len(self.waypoint_list)-1:
            self.wp_xyz_next = np.array([[0.],[0.],[0.]])
        else:
            self.wp_next += 1
            self.wp_xyz_next = self.waypoint_list[self.wp_next]

    # check collision for input point with obstacles
    def check_collision(self, pos, offset):
        cols = [np.linalg.norm(pos-s.get_center()) <= offset+s.get_radius() for s in self.obstacles]
        return sum(cols) > 0
        
    def rotate_translate_pts(self, pos):
        quat = self.iris.get_q()
        Q_inv = self.q_mult(self.q_conj(quat))
        Q = quat
        rotation = [Q_inv.dot(self.q_mult(xyz).dot(Q))[1:] for xyz in self.pts]
        translation = [r+pos for r in rotation]
        return translation

    def add_waypoint(self, waypoint):
        self.waypoint_list.append(waypoint)

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
        curr_dist = xyz-self.wp_xyz+self.datum
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
        dist_rew = 100.*(self.dist_norm-dist_hat)
        att_rew = 10.*((self.att_norm_sin-att_hat_sin)+(self.att_norm_cos-att_hat_cos))
        vel_rew = 10.*(self.vel_norm-vel_hat)
        ang_rew = 10.*(self.ang_norm-ang_hat)
        self.dist_norm = dist_hat
        self.att_norm_sin = att_hat_sin
        self.att_norm_cos = att_hat_cos
        self.vel_norm = vel_hat
        self.ang_norm = ang_hat
        self.vec_xyz = curr_dist
        self.vec_zeta_sin = curr_att_sin
        self.vec_zeta_cos = curr_att_cos
        self.vec_pqr = curr_ang
        cmplt_rew = 0.
        if self.dist_norm <= self.goal_thresh:
            cmplt_rew += (self.waypoints_reached+1)*100.
            self.waypoint_achieved(xyz)
            curr_dist = xyz-self.wp_xyz+self.datum
            dist_hat = np.linalg.norm(curr_dist)
            self.dist_norm = dist_hat
        col_rew = 0.
        if self.pol_col:
            col_rew -= 100
            
        # agent gets a negative reward for excessive action inputs
        ctrl_rew = 0.
        ctrl_rew -= np.sum(((action-self.trim_np)/self.action_bound[1])**2)
        ctrl_rew -= np.sum((((action-self.prev_action)/self.action_bound[1])**2))
        ctrl_rew -= np.sum((uvw-self.prev_uvw)**2)
        ctrl_rew -= np.sum((pqr-self.prev_pqr)**2)

        # agent gets a slight negative reward for time spent in flight
        time_rew = 0.
        return dist_rew, att_rew, vel_rew, ang_rew, ctrl_rew, time_rew, cmplt_rew, col_rew

    def policy_terminal(self, pos):
        xyz, _, _, _ = pos
        mask1 = np.abs(xyz[0]) > self.length
        mask2 = np.abs(xyz[1]) > self.width
        mask3 = np.abs(xyz[2]) > self.height
        if mask1 or mask2 or mask3:
            return True
        elif self.pol_col:
            return True
        elif self.t*self.ctrl_dt >= self.T*(1+self.waypoints_reached):
            return True
        elif np.linalg.norm(xyz-self.goal_xyz) <= self.goal_thresh:
            if self.waypoints_reached == len(self.waypoint_list):
                print("Final goal achieved!")
            return True
        else:
            return False

    def planner_terminal(self, args):
        xyz = args
        mask1 = np.abs(xyz[0]) > self.length
        mask2 = np.abs(xyz[1]) > self.width
        mask3 = np.abs(xyz[2]) > self.height
        if mask1 or mask2 or mask3:
            return True
        if np.linalg.norm(self.vec_xyz_wp) < self.goal_thresh:
            print("Goal reached in {} steps".format(self.count))
            return True
        if self.count >= self.steps_wp:
            return True
        if self.pla_col:
            return True
        return False

    def planner_reward(self, args):
        next_state = args
        d1 = np.linalg.norm(self.waypoint-self.goal_xyz)
        vec_xyz = next_state-self.goal_xyz
        guide_rew = 1./np.linalg.norm(vec_xyz)**2
        cmplt_rew = 0.
        if np.linalg.norm(vec_xyz) <= self.goal_thresh:
            cmplt_rew = 500.
        mask1 = np.abs(next_state[0]) > self.length
        mask2 = np.abs(next_state[1]) > self.width
        mask3 = np.abs(next_state[2]) > self.height
        oob_rew = 0.
        if mask1 or mask2 or mask3:
            oob_rew -= 500.
        col_rew = 0.
        if self.pla_col:
            col_rew -= 500.
        d2 = np.linalg.norm(vec_xyz)
        dist_rew = 100.*(d1-d2)
        step_rew = -2.
        self.vec_xyz_wp = vec_xyz
        return dist_rew, guide_rew, cmplt_rew, step_rew, oob_rew, col_rew

    def planner_step(self,action):
        def process_action(action):
            action_mag = np.linalg.norm(action)
            if action_mag != 0.:
                if action_mag > self.max_step:
                    return (action*(self.max_step/action_mag)).reshape((3,1))
                else:
                    return action.reshape((3,1))
            else:
                return np.zeros((3,1))

        command = process_action(action)
        next_waypoint = self.waypoint+command
        self.pla_col = self.check_collision(next_waypoint, self.col_rad)
        self.waypoint_list.append(next_waypoint.copy())
        info = self.planner_reward(next_waypoint)
        rew = sum(info)
        position_obs = sum([(next_waypoint-obs.xyz).T.tolist()[0]+[obs.rad] for obs in self.obstacles],[])
        done = self.planner_terminal(next_waypoint)
        next_state = next_waypoint.T.tolist()[0]+position_obs+self.vec_xyz_wp.T.tolist()[0]
        self.waypoint = next_waypoint
        self.count += 1
        return next_state, rew, done, {"dist_rew": info[0],
                                        "guide_rew": info[1],
                                        "cmplt_rew": info[2],
                                        "step_rew": info[3],
                                        "oob_rew": info[4],
                                        "col_rew": info[5]}

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
            xyz, zeta, uvw, pqr = self.iris.step(rpm_command)
        moved_pts = self.rotate_translate_pts(xyz)
        self.pol_col = sum([self.check_collision(p, 0.) for p in moved_pts]) > 0
        xyz -= self.datum
        sin_zeta = np.sin(zeta)
        cos_zeta = np.cos(zeta)
        current_rpm = (self.iris.get_rpm()/self.action_bound[1]).tolist()
        next_position = xyz.T.tolist()[0]
        next_attitude = sin_zeta.T.tolist()[0]+cos_zeta.T.tolist()[0]
        next_velocity = uvw.T.tolist()[0]+pqr.T.tolist()[0]
        next_state = next_position+next_attitude+next_velocity
        info = self.policy_reward((xyz, zeta, uvw, pqr), self.trim_np+action*self.bandwidth)
        done = self.policy_terminal((xyz, zeta, uvw, pqr))
        reward = sum(info)
        position_obs = sum([(xyz-obs.xyz).T.tolist()[0]+[obs.rad] for obs in self.obstacles],[])
        position_goal = self.vec_xyz.T.tolist()[0]
        attitude_goal = self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]
        goal = position_goal+attitude_goal
        next_position_goal = (xyz-self.wp_xyz_next).T.tolist()[0]
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
                                        "cmplt_rew": info[6],
                                        "col_rew": info[7]}

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
        self.pla_col = False
        self.waypoint_list = []
        self.count = 0
        self.waypoints_reached = 0
        self.wp_curr = 0
        self.wp_next = 1
        self.obstacles = self.generate_obstacles()
        self.goal_xyz = self.generate_goal()
        xyz, zeta, uvw, pqr = self.iris.reset()
        self.datum = xyz.copy()
        self.waypoint = xyz.copy()
        self.iris.set_rpm(np.array(self.trim))
        state = xyz.T.tolist()[0]
        vec = (xyz-self.goal_xyz).T.tolist()[0]
        position_obs = sum([(xyz-obs.xyz).T.tolist()[0]+[obs.rad] for obs in self.obstacles], [])
        return state+position_obs+vec, (xyz, zeta, uvw, pqr)

    def policy_reset(self, args):
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
        self.pol_col = False
        xyz, zeta, uvw, pqr = args
        self.t = 0
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
        self.vec_xyz = xyz-self.wp_xyz
        self.vec_zeta_sin = np.sin(zeta)-self.wp_zeta_sin
        self.vec_zeta_cos = np.cos(zeta)-self.wp_zeta_cos
        self.vec_uvw = uvw-self.wp_uvw
        self.vec_pqr = pqr-self.wp_pqr
        self.dist_norm = np.linalg.norm(self.vec_xyz)
        self.att_norm_sin = np.linalg.norm(self.vec_zeta_sin)
        self.att_norm_cos = np.linalg.norm(self.vec_zeta_cos)
        self.vel_norm = np.linalg.norm(self.vec_uvw)
        self.ang_norm = np.linalg.norm(self.vec_pqr)
        position_obs = sum([(xyz-obs.xyz).T.tolist()[0]+[obs.rad] for obs in self.obstacles],[])
        position_goal = self.vec_xyz.T.tolist()[0]
        attitude_goal = self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]
        goal = position_goal+attitude_goal
        next_position_goal = (xyz-self.wp_xyz_next).T.tolist()[0]
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
        self.ani.draw_goal(self.zero)
        #for p in self.pts:
        #    self.ani.draw_goal(p[1:], (0.5,0,0.5))  
        for i in range(len(self.waypoint_list)):
            wp = self.waypoint_list[i]
            if i == 0:
                self.ani.draw_line(self.zero, wp)
            else:
                self.ani.draw_line(wp, self.waypoint_list[i-1])
            self.ani.draw_goal(wp)
        for s in self.obstacles:
            self.ani.draw_sphere(s.get_center(), s.get_radius())
        self.ani.draw_goal(self.goal_xyz, color=(0.5,0,0))
        self.ani.draw_label("Time: {0:.2f}".format(self.t*self.ctrl_dt), 
            (self.ani.window.width // 2, 20.0))
        self.ani.draw()