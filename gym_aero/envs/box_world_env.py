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
from gym_aero.envs.Box_world_helper import *
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


"""Found a small bug, will fix and clean asap"""
"""--------------------------BUG------------------------"""
class BoxWorldEnv(gym.Env):

    def __init__(self):
        metadata = {'render.modes': ['human']}
        self.r_max = 2.5
        self.goal_thresh = 0.05
        self.t = 0
        self.T = 3.5
        self.action_space = np.zeros((4,))
        self.observation_space = np.zeros((34,))

        self.goal_xyz = self.generate_goal(self.r_max)
        self.goal_zeta_sin = np.sin(np.array([[0.],
                                            [0.],
                                            [0.]]))
        self.goal_zeta_cos = np.cos(np.array([[0.],
                                            [0.],
                                            [0.]]))

        self.spawn =np.array([[0.],
                                [0.],
                                [0.]])
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
        self.spawn= xyz
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
        self.sensor_total = 3
        self.sensor_spacing = 0.25
        self.circle_sensors=[]
        self.circle_sensors_adjusted = [] ##is adjusted to the quads orientaton
        for i in range(0,self.sensor_total):
            s = self.get_sensors((i+1+0.2)*self.sensor_spacing)
            self.circle_sensors.append(s)
        self.total_obstacles = 8
        self.obstacles = self.gen_obstacles()
        self.nearest_diatance =  self.get_nearest_obstacle(xyz)[0]
        self.sensor = Circle_sensors()
        self.sensor.update_sensor_location(xyz,zeta)
        self.fig = None
        self.axis3d = None

    def gen_obstacles(self):
        temp_sphere_list = []
        for i in range(0, self.total_obstacles):
            S = sphere_objects()
            spawn_dist = np.linalg.norm(S.get_center() - self.spawn)
            goal_dist = np.linalg.norm(S.get_center() - self.goal_xyz)
            if spawn_dist < 0.5:
                S.move_cords()
            if goal_dist < 0.5:
                S.move_cords()
            temp_sphere_list.append(S)
        return temp_sphere_list

    def get_nearest_obstacle(self,xyz):
        min_obj = None
        min_dist = 100
        for O in self.obstacles:
            dist = np.linalg.norm(xyz - O.get_center())

            if dist < min_dist:
                min_dist = dist
                min_obj = O

        return min_dist,min_obj

    def get_goal(self):
        return self.goal_xyz

    def reward(self, state, action,done):
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
        dist_rew = 100*(self.dist_norm-dist_hat)## + self.get_sigmoid_val(-0.5,self.iris.get_inertial_velocity()[2][0])
        att_rew = 10*((self.att_norm_sin-att_hat_sin)+(self.att_norm_cos-att_hat_cos))
        vel_rew = 0.1*(self.vel_norm-vel_hat)
        ang_rew = 0.1*(self.ang_norm-ang_hat)
        temp = self.get_nearest_obstacle(xyz)[0]
        obj_rew = 3*(temp - self.nearest_diatance)
        self.nearest_diatance = temp
        if(dist_rew > 0):
            self.distance_decrease = True
        else:
            self.distance_decrease = False
        # if done and self.t < self.T:
        #     obj_rew = -10000

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
        ctrl_rew = -np.sum(((action/self.action_bound[1])**2))

        # agent gets a positive reward for time spent in flight
        time_rew = 0.1

        #print("D: ",dist_rew,"AT: ",att_rew,'OB: ',obj_rew)
        return dist_rew, att_rew, vel_rew, ang_rew, ctrl_rew, time_rew, cmplt_rew,obj_rew


    def get_sensors(self,r):
        n = 20
        cir = np.array([[cos(2*np.pi/n*x)*r, sin(2*np.pi/n*x)*r, 0] for x in range(n+1)])
        cir += np.array([[0.0, 0.0, 0.0] for x in range(n+1)])
        return cir

    def update_sensors(self,xyz,p1):
        q = self.iris.get_q()
        Q_inv =  self.iris.q_conj(q)
        Q =  self.iris.q_mult(Q_inv)
        r =  self.R(Q_inv)
        adj_p1 = np.einsum('ij,kj->ki', r, p1)
        adj_p1 = np.matlib.repmat(xyz.T, 20+1,1)+adj_p1
        return adj_p1

    def R(self, p):
        p0, p1, p2, p3 = p[0,0], p[1,0], p[2,0], p[3,0]
        x11 = p0**2+p1**2-p2**2-p3**2
        x12 = 2.*(p1*p2-p0*p3)
        x13 = 2.*(p1*p3+p0*p2)
        x21 = 2.*(p1*p2+p0*p3)
        x22 = p0**2-p1**2+p2**2-p3**2
        x23 = 2.*(p2*p3-p0*p1)
        x31 = 2.*(p1*p3-p0*p2)
        x32 = 2.*(p2*p3+p0*p1)
        x33 = p0**2-p1**2-p2**2+p3**2
        return np.array([[x11, x12, x13],
                        [x21, x22, x23],
                        [x31, x32, x33]])

    def get_sigmoid_val(self,e_val,val):
        sig = 1/(1+math.exp(-val+e_val))
        dir_sig = 4*sig*(1-sig)
        return dir_sig

    def terminal(self, pos):
        xyz, zeta ,uvw= pos
        mask1 = 0#zeta[0:2] > pi/2
        mask2 = 0#zeta[0:2] < -pi/2
        mask3 = self.dist_norm > 5
        collision = False
        n_dist , n_obj = self.get_nearest_obstacle(xyz)
        if n_dist <= (n_obj.get_radius() + self.iris.get_prop_radius()):
            collision = True
        if np.sum(mask1) > 0 or np.sum(mask2) > 0 or np.sum(mask3) > 0:
            return True
        elif uvw[2][0] < -2.0:
            return True
        # elif xyz[0][0] < -3 or xyz[0][0] > 3:
        #     return True
        # elif xyz[1][0] < -3 or xyz[1][0] > 3:
        #     return True
        # elif  xyz[2][0] < -3 or xyz[2][0] > 3:
        #     return True
        elif collision:
            return True
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
        self.t += self.ctrl_dt
        self.sensor.update_sensor_location(xyz,zeta)
        sin_zeta = np.sin(zeta)
        cos_zeta = np.cos(zeta)
        a = (action/self.action_bound[1]).tolist()
        next_state = xyz.T.tolist()[0]+sin_zeta.T.tolist()[0]+cos_zeta.T.tolist()[0]+uvw.T.tolist()[0]+pqr.T.tolist()[0]
        self.circle_sensors_adjusted = []
        for p1 in self.circle_sensors:
            self.circle_sensors_adjusted.append(self.update_sensors(xyz,p1))
        # self.p1_adj = self.update_sensors(xyz,self.p1)
        done = self.terminal((xyz, zeta, uvw))
        info = self.reward((xyz, zeta, uvw, pqr), action,done)
        reward = sum(info)
        goals = self.vec_xyz.T.tolist()[0]+self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]+self.vec_uvw.T.tolist()[0]+self.vec_pqr.T.tolist()[0]
        next_state = next_state+a+goals
        return next_state, reward, done, info

    def reset(self):
        self.goal_achieved = False
        self.t = 0.
        xyz, zeta, uvw, pqr = self.iris.reset()
        self.iris.set_rpm(np.array(self.trim))
        self.goal_xyz = self.generate_goal(self.r_max)
        sin_zeta = np.sin(zeta)
        cos_zeta = np.cos(zeta)
        self.vec_xyz = xyz-self.goal_xyz
        self.vec_zeta_sin = sin_zeta
        self.vec_zeta_cos = cos_zeta
        self.vec_uvw = uvw
        self.vec_pqr = pqr
        #
        # for i in range(0,self.sensor_total):
        #     s = self.get_sensors((i+1)*self.sensor_spacing)
        #     self.circle_sensors.append(s)
        # self.circle_sensors_adjusted = self.circle_sensors
        self.obstacles = self.gen_obstacles()
        self.nearest_diatance =  self.get_nearest_obstacle(xyz)[0]
        a = (self.trim_np/self.action_bound[1]).tolist()
        goals = self.vec_xyz.T.tolist()[0]+self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]+self.vec_uvw.T.tolist()[0]+self.vec_pqr.T.tolist()[0]
        state = xyz.T.tolist()[0]+sin_zeta.T.tolist()[0]+cos_zeta.T.tolist()[0]+uvw.T.tolist()[0]+pqr.T.tolist()[0]+a+goals
        return state

    ##Check if the boxes and goal or spawn location overlap
    ##TODO:need to find a cleaner way of ensuring they are spaced out
    def check_overlap(self,b):
        max_int_dist = b.get_distance_to_corner()
        c = b.get_center()
        goal_dist =np.linalg.norm(self.goal_xyz - c)
        spawn_dist =  np.linalg.norm(self.spawn - c)

        if(goal_dist > 1.25*max_int_dist and spawn_dist > 1.25*max_int_dist):
            # for ob in self.boxes:
            #     print('ob')
            #     dist_to_box = np.linalg.norm(ob.get_center()-c)
            #     if(dist_to_box < max_int_dist):
            #         return False
            return True
        else:
            return False

    def generate_goal(self, r_max):
        r = np.random.uniform(low=0.75, high=r_max)
        phi = random.uniform(-2*pi, 2*pi)
        theta = random.uniform(-2*pi, 2*pi)
        x = r*sin(theta)*cos(phi)
        y = r*sin(theta)*sin(phi)
        z = r*cos(theta)
        return np.array([[2],
                        [2],
                        [2]])

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
        for O in self.obstacles:
            O.draw_object(self.axis3d)
        xyz, zeta, uvw, pqr = self.iris.get_state()
        # if self.distance_decrease:
        #     self.axis3d.plot([self.goal_xyz[0][0],xyz[0][0]],[self.goal_xyz[1][0],xyz[1][0]],[self.goal_xyz[2][0],xyz[2][0]],c='g')
        # else:
        #     self.axis3d.plot([self.goal_xyz[0][0],xyz[0][0]],[self.goal_xyz[1][0],xyz[1][0]],[self.goal_xyz[2][0],xyz[2][0]],c='r')
        n_obj = self.get_nearest_obstacle(xyz)[1]
        self.axis3d.plot([n_obj.get_center()[0][0],xyz[0][0]],[n_obj.get_center()[1][0],xyz[1][0]],[n_obj.get_center()[2][0],xyz[2][0]],c='b')
        #self.sensor.draw_sensors(self.axis3d)
        for p1 in self.circle_sensors_adjusted:
            self.axis3d.plot(p1[:,0], p1[:,1], p1[:,2],'k')
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
