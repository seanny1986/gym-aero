import simulation.quadrotor3 as quad
import simulation.config as cfg
import numpy as np
import random
from math import pi, sin, cos, exp
import gym
from gym import error, spaces, utils
from gym.utils import seeding
import simulation.animation_gl as ani_gl

class PerchMTEnv(gym.Env):
    """
        Environment wrapper for training low-level flying skills. In this environment, the aircraft
        is meant to perch on any of the four walls at a 90 degree angle
    """
    def __init__(self):
        metadata = {'render.modes': ['human']}
        self.r_max = 2.5
        self.goal_thresh = 0.2
        self.t = 0
        self.T = 10
        self.action_space = np.zeros((4,))
        self.observation_space = np.zeros((34,))

        #Dictionary to hold the wall data
        self.wall_data = {"wall_plane":None,"PR":None,"ELA":None,"wall_pos":None}
        wall_goal = self.get_wall_goal()
        self.wall =wall_goal[0]
        self.goal_xyz = wall_goal[1]
        zeta_goal  = self.get_goal_zeta()
        self.goal_zeta_sin = np.sin(zeta_goal)
        self.goal_zeta_cos = np.cos(zeta_goal)

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
        self.vec_xyz = xyz-self.goal_xyz
        self.vec_zeta_sin = np.sin(zeta)-self.goal_zeta_sin
        self.vec_zeta_cos = np.cos(zeta)-self.goal_zeta_cos
        self.vec_uvw = uvw-self.goal_uvw
        self.vec_pqr = pqr-self.goal_pqr
        self.vel_r = pqr[2][0]
        exp_wall_approach_angle = self.wall_data["ELA"]
        if self.wall_data["PR"] == "R":
            self.wall_approach_angle = abs(exp_wall_approach_angle - zeta[0][0])+ abs(0 - zeta[1][0])
            self.approach_line = ((self.goal_xyz[0][0]-xyz[0][0])**2+(self.goal_xyz[2][0]-xyz[2][0])**2)**0.5

        else:
            self.wall_approach_angle = abs(exp_wall_approach_angle - zeta[1][0]) + abs(0 - zeta[0][0])
            self.approach_line = ((self.goal_xyz[1][0]-xyz[1][0])**2+(self.goal_xyz[2][0]-xyz[2][0])**2)**0.5
        self.dist_norm = np.linalg.norm(self.vec_xyz)
        self.att_norm_sin = np.linalg.norm(self.vec_zeta_sin)
        self.att_norm_cos = np.linalg.norm(self.vec_zeta_cos)
        self.vel_norm = np.linalg.norm(self.vec_uvw)
        self.ang_norm = np.linalg.norm(self.vec_pqr)
        self.init_rendering = False

    def get_goal(self):
        return self.goal_xyz

    def get_wall_goal(self):
        """
        sets a wall in either 4 corners of of the world
        The goal is anywehere on this wall, the location of the wall
        and goal is randomly generated per reset
        - Landing is more consistent if the landing pad is not randomly
          located on each of the walls. Only randomness would be the wall.

        - PR = pitch or roll, needs to be controlled
        - ELA = expected landing angle, ie -90 or +90
        - wall pos = is the location of the wall
        - wall plane = X or Y axis contains the wall 
        """

        size = 6
        if random.randint(0,1):
            x = -3
            if random.randint(0,1):
                y = -3
                self.wall_data["ELA"] = np.pi/2
                self.wall_data["wall_pos"] = y
            else:
                y = 3
                self.wall_data["ELA"] = -np.pi/2
                self.wall_data["wall_pos"] = y

            g_x = 0#random.randint(-5,5)/10.  #Comment this out for more consistency
            g_z = 2.5
            goal = np.array([[g_x],[y],[g_z]])
            self.wall_data["wall_plane"] = 'y'
            self.wall_data["PR"] = 'R'
            A1 = np.array([x,y,-3])
            A2 = np.array([x+size,y,-3])
            A3 = np.array([x+size,y,3])
            A4 = np.array([x,y,3])
        else:
            y = -3
            if random.randint(0,1):
                x = -3
                self.wall_data["ELA"] = -np.pi/2
                self.wall_data["wall_pos"] = x
            else:
                x = 3
                self.wall_data["ELA"] = np.pi/2
                self.wall_data["wall_pos"] = x

            g_y = 0#random.randint(-5,5)/10. #Comment this out for more consistency
            g_z = 2.5
            goal = np.array([[x],[g_y],[g_z]])
            self.wall_data["wall_plane"] = 'x'
            self.wall_data["PR"] = 'P'
            A1 = np.array([x,y,-3])
            A2 = np.array([x,y+size,-3])
            A3 = np.array([x,y+size,3])
            A4 = np.array([x,y,3])
        return ([[A1,A2,A3,A4]],goal)

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
        dist_rew = 100.*(self.dist_norm-dist_hat)
        att_rew = 1.*((self.att_norm_sin-att_hat_sin)+(self.att_norm_cos-att_hat_cos))
        vel_rew = 1.*(self.vel_norm-vel_hat)
        ang_rew = 1.*(self.ang_norm-ang_hat)

        #Changes the reward based on which wall the quad needs to land on
        exp_wall_approach_angle = self.wall_data["ELA"]
        wall_approach_angle_new = 0
        if self.wall_data["PR"] == "R":
            wall_approach_angle_new = abs(exp_wall_approach_angle - zeta[0][0]) + abs(0 - zeta[1][0])
            approach_line_new = ((self.goal_xyz[0][0]-xyz[0][0])**2+(self.goal_xyz[2][0]-xyz[2][0])**2)**0.5
        else:
            wall_approach_angle_new = abs(exp_wall_approach_angle - zeta[1][0]) + abs(0 - zeta[0][0])
            approach_line_new = ((self.goal_xyz[1][0]-xyz[1][0])**2+(self.goal_xyz[2][0]-xyz[2][0])**2)**0.5
        wall_approach_angle_diff = self.wall_approach_angle - wall_approach_angle_new
        dist_diff = self.dist_norm-dist_hat
        wall_approach_angle_rew = 150*(abs(dist_diff))*wall_approach_angle_diff
        approach_line_rew =  5*(self.approach_line - approach_line_new)
        self.approach_line = approach_line_new
        self.wall_approach_angle = wall_approach_angle_new
        if self.goal_met(xyz,zeta):
            cmplt_rew = 200
        else:
            cmplt_rew = 0
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
        ctrl_rew = -np.sum(((action/self.action_bound[1])**2))

        # agent gets a positive reward for time spent in flight
        time_rew = 0.
        return dist_rew, att_rew, vel_rew, ang_rew, ctrl_rew, time_rew, cmplt_rew, wall_approach_angle_rew

    def get_sigmoid_val(self,e_val,val):
        sig = 1/(1+exp(-val+e_val))
        dir_sig = 4*sig*(1-sig)
        return dir_sig

    def check_wall_collision(self,xyz,zeta):
        """
        Checks if quad crashed into any of the walls,
        world is bounded by all 4 walls located at +-3

        """

        if(xyz[0][0] > 3 or xyz[0][0] < -3):
            return True
        elif (xyz[1][0] > 3 or xyz[1][0] < -3):
            return True
        else:
            return False

    def goal_met(self,xyz,zeta):
        """
        Checks to see if the goal has been met
        - Quad has to be within 10 degrees of its goal landing angle
        - Quad has to be within 0.3m of the pad in order to be hooked
        - There is a 0.15 error allowance for the hook
        """

        wall_approach_angle_new = 0
        if self.wall_data["PR"] == "R":
            wall_approach_angle_new = abs( self.wall_data["ELA"] - zeta[0][0])
        else:
            wall_approach_angle_new = abs( self.wall_data["ELA"]- zeta[1][0])
        if self.wall_data["wall_plane"] == "x":
            hook_tol = ((xyz[1][0]-self.goal_xyz[1][0])**2 +(xyz[2][0]-self.goal_xyz[2][0])**2)**0.5
            hook_len = abs(xyz[0][0]-self.wall_data["wall_pos"])
        else:
            hook_tol = ((xyz[0][0]-self.goal_xyz[0][0])**2 +(xyz[2][0]-self.goal_xyz[2][0])**2)**0.5
            hook_len = abs(xyz[1][0]-self.wall_data["wall_pos"])
        if hook_len < 0.3 and hook_tol <0.15 and wall_approach_angle_new <= 0.1745:#0.087:
            return True
        else:
            return False

    def terminal(self, pos):
        """
        Sim can end on the following conditions
        - Time runs out
        - Goal is acheived
        - Quad crashes into wall
        - Outside world
        """

        xyz, zeta = pos
        mask1 = 0
        mask2 = 0
        mask3 = self.dist_norm > 5
        if np.sum(mask1) > 0 or np.sum(mask2) > 0 or np.sum(mask3) > 0:
            return True
        elif self.goal_met(xyz,zeta):
           return True
        elif self.check_wall_collision(xyz,zeta):
            return True
        elif self.t >= self.T:
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
        self.t += self.ctrl_dt
        sin_zeta = np.sin(zeta)
        cos_zeta = np.cos(zeta)
        current_rpm = (self.iris.get_rpm()/self.action_bound[1]).tolist()
        next_position = xyz.T.tolist()[0]
        next_attitude = sin_zeta.T.tolist()[0]+cos_zeta.T.tolist()[0]
        next_velocity = uvw.T.tolist()[0]+pqr.T.tolist()[0]
        next_state = next_position+next_attitude+next_velocity
        done = self.terminal((xyz, zeta))
        info = self.reward((xyz, zeta, uvw, pqr), action,done)
        reward = sum(info)
        position_goal = self.vec_xyz.T.tolist()[0] 
        attitude_goal = self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]
        velocity_goal = self.vec_uvw.T.tolist()[0]+self.vec_pqr.T.tolist()[0]
        goals = position_goal+attitude_goal+velocity_goal
        next_state = next_state+current_rpm+goals
        return next_state, reward, done, {"info": info}

    def get_goal_zeta(self):
        yaw = [0.]
        roll = [0.]
        pitch = [0.]
        exp_ang = self.wall_data ["ELA"]
        if self.wall_data["PR"] == "R":
            roll[0] = exp_ang
        else:
            pitch[0] = exp_ang
        return np.array([roll,pitch,yaw])

    def reset(self):
        self.goal_achieved = False
        self.t = 0.

        #Holds info about the location of wall and expected landing angle
        self.wall_data = {"wall_plane":None,"PR":None,"ELA":None,"wall_pos":None}
        wall_goal = self.get_wall_goal()
        xyz, zeta, uvw, pqr = self.iris.reset()
        self.iris.set_state(xyz,zeta,uvw,pqr)
        self.iris.set_rpm(np.array(self.trim))
        self.wall = wall_goal[0]
        self.goal_xyz = wall_goal[1]
        goal_zeta = self.get_goal_zeta()

        sin_zeta = np.sin(goal_zeta)
        cos_zeta = np.cos(goal_zeta)
        self.goal_zeta_sin = sin_zeta
        self.goal_zeta_cos = cos_zeta
        self.vec_xyz = xyz-self.goal_xyz
        sin_zeta = np.sin(zeta)
        cos_zeta = np.cos(zeta)
        self.vec_zeta_sin = sin_zeta
        self.vec_zeta_cos = cos_zeta
        exp_wall_approach_angle = self.wall_data["ELA"]
        if self.wall_data["PR"] == "R":
            self.wall_approach_angle = abs(exp_wall_approach_angle - zeta[0][0])+ abs(0 - zeta[1][0])
            self.approach_line = ((self.goal_xyz[0][0]-xyz[0][0])**2+(self.goal_xyz[2][0]-xyz[2][0])**2)**0.5
        else:
            self.wall_approach_angle = abs(exp_wall_approach_angle - zeta[1][0]) + abs(0 - zeta[0][0])
            self.approach_line = ((self.goal_xyz[1][0]-xyz[1][0])**2+(self.goal_xyz[2][0]-xyz[2][0])**2)**0.5
        self.att_norm_sin = np.linalg.norm(self.vec_zeta_sin)
        self.att_norm_cos = np.linalg.norm(self.vec_zeta_cos)
        self.vec_uvw = uvw-self.goal_uvw
        self.vec_pqr = pqr-self.goal_pqr
        self.vel_norm = np.linalg.norm(self.vec_uvw)
        self.ang_norm = np.linalg.norm(self.vec_pqr)
        self.dist_norm = np.linalg.norm(self.vec_xyz)
        self.vec_uvw = uvw
        self.vec_pqr = pqr
        a = (self.trim_np/self.action_bound[1]).tolist()
        goals = self.vec_xyz.T.tolist()[0]+self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]+self.vec_uvw.T.tolist()[0]+self.vec_pqr.T.tolist()[0]
        state = xyz.T.tolist()[0]+sin_zeta.T.tolist()[0]+cos_zeta.T.tolist()[0]+uvw.T.tolist()[0]+pqr.T.tolist()[0]+a+goals
        return state

    def render(self, mode='human', close=False):
        if not self.init_rendering:
            self.ani = ani_gl.VisualizationGL(name="Perch")
            self.init_rendering = True
        self.ani.draw_quadrotor(self.iris)
        self.ani.draw_goal(self.goal_xyz)
        self.ani.draw_label("Time: {0:.2f}".format(self.t*self.ctrl_dt), 
            (self.ani.window.width // 2, 20.0))
        self.ani.draw()
