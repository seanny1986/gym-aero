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


class BoxWorld(gym.Env):

    def __init__(self):
        metadata = {'render.modes': ['human']}
        self.r_max = 2.5
        self.goal_thresh = 0.05
        self.t = 0
        self.T = 10
        self.action_space = np.zeros((4,))
        self.observation_space = np.zeros((34,))

        self.goal_xyz = self.generate_goal(self.r_max)
        self.goal_zeta_sin = np.sin(np.array([[0.],
                                            [0.],
                                            [0.]]))
        self.goal_zeta_cos = np.cos(np.array([[0.],
                                            [0.],
                                            [0.]]))

        self.spawn =np.array([[2],
                                [2],
                                [-2]])
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

        self.sensor_total = 3 ##This is pretty much the max before big preformance reduction
        self.sensor_spacing = 0.25
        self.circle_sensors=[]
        self.obstacles = []
        self.total_obstacles = 5
        self.largest_sensor = 0

        ##This needs to be added to quadrotor3.py
        self.prop_radius = self.iris.get_prop_radius()

        self.gen_sensors()
        self.gen_obstacles()

        self.obstacle_rew_val = self.get_obstacle_rew(xyz)

        self.fig = None
        self.axis3d = None

    #Generates the sensors
    #Level 0 is the smallest circle, used for collison and termination
    #Next size up is level 1 and so on
    def gen_sensors(self):
        radius = 0
        for i in range(0,self.sensor_total):
            radius = i+1+0.15
            s = Circle_sensors((radius)*self.sensor_spacing,i)
            self.circle_sensors.append(s)

        ##Holds the radius of the largest circle
        self.largest_sensor = radius

    #Generates a set number of shpere obstacles
    ##Ensures spheres arent overlapping with goal or quads spawn
    #Quad can overlap with itself though
    def gen_obstacles(self):
        for i in range(0,self.total_obstacles):

            rad = random.randint(4,7)/10
            c = Sphere_Object(rad)
            doLoop = True
            while(doLoop):
                dist_to_goal = np.linalg.norm(np.array(c.get_center())-self.goal_xyz)
                dist_to_spawn =np.linalg.norm(np.array(c.get_center())-self.spawn)
                required_spacing = self.prop_radius + c.get_radius()
                if dist_to_goal <= required_spacing or dist_to_spawn <= required_spacing:
                    if random.randint(0,1):
                        c.move_x(-2*self.prop_radius)
                        c.move_y(2*self.prop_radius)
                    else:
                        c.move_x(2*self.prop_radius)
                        c.move_y(-2*self.prop_radius)
                else:
                    doLoop = False
            self.obstacles.append(c)

    def get_obstacles(self):
        return self.obstacles()

    #Checks if object is closer than the smallest circle
    #i.e if distance to object is less than any point on level 0 circle
    def check_collision(self,xyz):
        c_lvl1 = None

        for c in self.circle_sensors:
            if c.get_level() == 0:
                c_lvl1 = c
                break

        for obj in self.obstacles:
            ##Only check for collision if object is closer than largest sensor
            dist = np.linalg.norm(xyz-obj.get_center())
            if dist < (self.largest_sensor + obj.get_radius()):
                for point in c_lvl1.get_points():
                    center = obj.get_center()
                    dist = ((point[0]-center[0])**2+(point[1]-center[1])**2+(point[2]-center[2])**2)**0.5
                    if dist < obj.get_radius():
                        return True

        return False


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



        # obj_temp  = self.get_obstacle_rew() - self.obstacle_rew_val
        # obj_rew = 1*obj_temp
        #self.obstacle_rew_val = obj_temp
        obj_rew = self.get_obstacle_rew(xyz)




        if(dist_rew > 0):
            self.distance_decrease = True
        else:
            self.distance_decrease = False


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




    ##Returns a value based on which circle the object interspets
    ##Each ring has a factor of 10 decrease from the prevois
    """This function is O^n is all cases and slowing things down"""
    # def get_obstacle_rew(self,xyz):
    #     obj_rew_temp = 0
    #     for obj in self.obstacles:
    #         for sensor in self.circle_sensors:
    #             min_dist = sensor.calculate_distance(obj.get_center())
    #             if min_dist < obj.get_radius():
    #                 level_penalty = (self.sensor_total-1) - sensor.get_level()
    #                 obj_rew_temp += -1*(10**level_penalty)
    #
    #     return obj_rew_temp


    """Same as above function is O(n) is most cases"""
    ##Only check the object for a collision if its closer than the largest circle_sensor
    ##This reduces the amount of checking. This method still takes into account the
    ##orientation of the craft and the sensors. However if an object is within the
    #distance of the largest circle it is added to a list and checked for a collision.
    def get_obstacle_rew(self,xyz):
        obj_rew_temp = 0

        temp_obj_list = []
        for obj in self.obstacles:
            dist = np.linalg.norm(xyz-obj.get_center())
            if dist < (self.largest_sensor + obj.get_radius()):
                temp_obj_list.append(obj)

        for obj in temp_obj_list:
            for sensor in self.circle_sensors:
                min_dist = sensor.calculate_distance(obj.get_center())
                if min_dist < obj.get_radius():
                    level_penalty = (self.sensor_total-1) - sensor.get_level()
                    obj_rew_temp += -1*(10**level_penalty)

        return obj_rew_temp



    def terminal(self, pos):
        xyz, zeta ,uvw= pos
        mask1 = 0#zeta[0:2] > pi/2
        mask2 = 0#zeta[0:2] < -pi/2
        mask3 = self.dist_norm > 7


        if np.sum(mask1) > 0 or np.sum(mask2) > 0 or np.sum(mask3) > 0:
            return True
        elif self.check_collision(xyz):
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


        sin_zeta = np.sin(zeta)
        cos_zeta = np.cos(zeta)
        a = (action/self.action_bound[1]).tolist()
        next_state = xyz.T.tolist()[0]+sin_zeta.T.tolist()[0]+cos_zeta.T.tolist()[0]+uvw.T.tolist()[0]+pqr.T.tolist()[0]

        for p1 in self.circle_sensors:
            p1.update_sensors(xyz,self.iris)

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
        self.vec_zeta_sin = np.sin(zeta)-self.goal_zeta_sin
        self.vec_zeta_cos = np.cos(zeta)-self.goal_zeta_cos
        self.att_norm_sin = np.linalg.norm(self.vec_zeta_sin)
        self.att_norm_cos = np.linalg.norm(self.vec_zeta_cos)
        self.vec_uvw = uvw-self.goal_uvw
        self.vec_pqr = pqr-self.goal_pqr
        self.vel_norm = np.linalg.norm(self.vec_uvw)
        self.ang_norm = np.linalg.norm(self.vec_pqr)
        self.dist_norm = np.linalg.norm(self.vec_xyz)
        self.vec_uvw = uvw
        self.vec_pqr = pqr

        self.obstacles = []
        self.circle_sensors = []
        self.gen_sensors()
        self.gen_obstacles()
        self.obstacle_rew_val = self.get_obstacle_rew(xyz)


        a = (self.trim_np/self.action_bound[1]).tolist()
        goals = self.vec_xyz.T.tolist()[0]+self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]+self.vec_uvw.T.tolist()[0]+self.vec_pqr.T.tolist()[0]
        state = xyz.T.tolist()[0]+sin_zeta.T.tolist()[0]+cos_zeta.T.tolist()[0]+uvw.T.tolist()[0]+pqr.T.tolist()[0]+a+goals
        return state



    def generate_goal(self, r_max):
        r = np.random.uniform(low=2, high=r_max)
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


        xyz, zeta, uvw, pqr = self.iris.get_state()
        # if self.distance_decrease:
        #     self.axis3d.plot([self.goal_xyz[0][0],xyz[0][0]],[self.goal_xyz[1][0],xyz[1][0]],[self.goal_xyz[2][0],xyz[2][0]],c='g')
        # else:
        #     self.axis3d.plot([self.goal_xyz[0][0],xyz[0][0]],[self.goal_xyz[1][0],xyz[1][0]],[self.goal_xyz[2][0],xyz[2][0]],c='r')

        #n_obj = self.get_nearest_obstacle(xyz)[1]

        #self.axis3d.plot([n_obj.get_center()[0][0],xyz[0][0]],[n_obj.get_center()[1][0],xyz[1][0]],[n_obj.get_center()[2][0],xyz[2][0]],c='b')
        #self.sensor.draw_sensors(self.axis3d)

        for c in self.obstacles:
            c.plot_sphere(self.axis3d)



        for p1 in self.circle_sensors:
            p1.plot(self.axis3d)



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
