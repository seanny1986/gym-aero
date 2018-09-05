import simulation.quadrotor3 as quad
import simulation.config as cfg
import simulation.animation as ani
import matplotlib.pyplot as pl
import mpl_toolkits.mplot3d.art3d as art3d
import numpy as np
import random
from math import pi, sin, cos
import gym
from gym import error, spaces, utils
from gym.utils import seeding



class LandParaEnv(gym.Env):
    """
        Environment wrapper for training low-level flying skills. The aircraft is required to land
        at a random positon. It is required to lower onto the landing pad, i.e the angle
        between the aircraft and pad is 90 deg.
    """
    def __init__(self):
        metadata = {'render.modes': ['human']}
        self.r_max = 2.5
        self.goal_thresh = 0.05
        self.t = 0
        self.T = 10
        self.action_space = np.zeros((4,))
        self.observation_space = np.zeros((34,))

        self.goal_xyz = np.array([[random.uniform(-3,3)],
                                [random.uniform(-3,3)],
                                [random.uniform(-3,0)]])
        self.spawn = np.array([[random.uniform(-3,3)],
                                [random.uniform(-3,3)],
                                [random.uniform(0,3)]])

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
        self.bandwidth = 30.

        # define bounds here
        self.xzy_bound = 0.5
        self.zeta_bound = pi/3
        self.uvw_bound = 0.5
        self.pqr_bound = 0.25

        self.iris.set_state(self.spawn, np.arcsin(self.goal_zeta_sin), self.goal_uvw, self.goal_pqr)
        self.iris.set_rpm(np.array(self.trim))
        xyz, zeta, uvw, pqr = self.iris.get_state()

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
        #self.landing_angle = (180/np.pi)*abs(np.arcsin(temp_O/dist_hat))[0])-90
        self.fig = None
        self.axis3d = None


        self.update_path()
        # print(self.x_para)
        # print(self.y_para)
        # print(self.z_para)

    def update_path(self):
        x_min,x_max = self.get_valid_x_range()
        self.x_para = []
        self.y_para = []
        self.z_para = []
        i = x_min
        #print(self.goal_xyz.T.tolist(),self.goal_xyz[0],self.get_y_z(self.goal_xyz[0]))
        #print('TEST ',self.spawn.T.tolist(),x_min,x_max,self.get_y_z(x_min),self.get_y_z(x_max))
        while i <= x_max:
            self.x_para.append(i)
            y_t,z_t = self.get_y_z(i)
            self.y_para.append(y_t)
            self.z_para.append(z_t)
            i = i + 0.1

        #print('SPAWN ',self.spawn.T.tolist(),self.x_para[0],self.y_para[0],self.z_para[0])


    def get_valid_x_range(self):
        if self.goal_xyz[0][0] < self.spawn[0][0]:

            return (self.goal_xyz[0][0],self.spawn[0][0])
        else:
            return (self.spawn[0][0],self.goal_xyz[0][0])

    def get_y_z(self,x):
        x_d = self.spawn[0][0]
        y_d = self.spawn[1][0]
        z_d = self.spawn[2][0]

        x_h = self.goal_xyz[0][0]
        y_h = self.goal_xyz[1][0]
        z_h = self.goal_xyz[2][0]

        if x_d > x_h:
            if x >= x_h and x <= x_d:
                m = (y_d-y_h)/(x_d-x_h)
                c = y_h - m*x_h
                y = m*x + c

                a = (z_h - z_d)/(x_h - x_d)**2
                z = a* (x - x_d)**2 + z_d

                return (y,z)
            else:
                return (None,None)
        else:
            if x <= x_h and x >= x_d:
                m = (y_d-y_h)/(x_d-x_h)
                c = y_h - m*x_h
                y = m*x + c

                a = (z_h - z_d)/(x_h - x_d)**2
                z = a* (x - x_d)**2 + z_d
                return (y,z)
            else:
                return (None,None)


    def get_dist(self,xyz):
        x_d = self.spawn[0][0]
        y_d = self.spawn[1][0]
        z_d = self.spawn[2][0]

        x_h = self.goal_xyz[0][0]
        y_h = self.goal_xyz[1][0]
        z_h = self.goal_xyz[2][0]

        x_c,y_c,z_c = xyz

        a = (z_h - z_d)/(x_h - x_d)**2


        temp = (z_c/a)-z_d
        if temp < 0:
            d = 10*(abs(x_c-x_h)**2 + abs(y_c-y_h)**2 + (abs(z_c-z_h)))**0.5
            return d[0]
        else:
            x = temp**0.5 + x_d
            m = (y_d-y_h)/(x_d-x_h)
            c = y_h - m*x_h
            y = m*x + c
            d = (abs(x_c-x)**2 + abs(y_c-y)**2)**0.5
            return d[0]






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

        # agent gets a negative reward based on how far away it is from the desired goal state
        dist_rew = 0.1*(1/(xyz[2] - self.goal_xyz[2]))[0]
        att_rew = 1*((self.att_norm_sin-att_hat_sin)+(self.att_norm_cos-att_hat_cos))
        #temp_O = xyz[2] - self.goal_xyz[2]

        #land_angle_rew = ((180/np.pi)*abs(np.arcsin(temp_O/dist_hat))[0] -90)
        la_temp = 10* 1/self.get_dist(xyz)
        land_angle_rew = la_temp

        #print(land_angle_rew)

        ##//TODO: Fix landing velocitiy
        vel_rew = 0*(self.vel_norm-vel_hat)
        ang_rew = 0*(self.ang_norm-ang_hat)

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

        # if self.dist_norm <= self.goal_thresh:
        #     cmplt_rew = 100.
        # else:
        #     cmplt_rew = 0

        # agent gets a negative reward for excessive action inputs
        ctrl_rew = -np.sum(((action/self.action_bound[1])**2))

        # agent gets a positive reward for time spent in flight
        time_rew = 0#-0.1
        ##print("D: ",dist_rew,"AT: ",att_rew,"LA: ",land_angle_rew )
        return dist_rew, att_rew, vel_rew, ang_rew, ctrl_rew, time_rew, land_angle_rew

    def terminal(self, pos):
        xyz, zeta = pos
        mask1 = zeta[0:2] > pi/2
        mask2 = zeta[0:2] < -pi/2
        mask3 = self.dist_norm > 5
        goal_dist = np.linalg.norm(xyz - self.goal_xyz)

        if np.sum(mask1) > 0 or np.sum(mask2) > 0 or np.sum(mask3) > 0:
            return True
        elif goal_dist < 0.1:
            print("Goal Achieved!")
            return True
        elif self.t >= self.T:
            print("Sim time reached: {:.2f}s".format(self.t))
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
        info = self.reward((xyz, zeta, uvw, pqr), action)
        done = self.terminal((xyz, zeta))
        reward = sum(info)
        ##sprint("REWARD ",reward)
        goals = self.vec_xyz.T.tolist()[0]+self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]+self.vec_uvw.T.tolist()[0]+self.vec_pqr.T.tolist()[0]
        next_state = next_state+a+goals
        return next_state, reward, done, info


    def get_goal(self):
        return self.goal_xyz

    def reset(self):

        self.goal_achieved = False
        self.t = 0.


        self.goal_xyz = np.array([[random.uniform(-3,0)],
                                [random.uniform(-3,0)],
                                [-2]])
        self.spawn = np.array([[random.uniform(0,3)],
                                [random.uniform(0,3)],
                                [2]])
        print(self.goal_xyz,self.spawn)
        self.iris.set_state(self.spawn, np.arcsin(self.goal_zeta_sin), self.goal_uvw, self.goal_pqr)
        self.iris.set_rpm(np.array(self.trim))
        xyz, zeta, uvw, pqr = self.iris.get_state()
        self.update_path()

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
        self.axis3d.plot(self.x_para,self.y_para,self.z_para)


        cir = pl.Circle((self.goal_xyz[0],self.goal_xyz[1]), 0.5)
        self.axis3d.add_patch(cir)
        art3d.pathpatch_2d_to_3d(cir, z=self.goal_xyz[2], zdir="z")
        self.axis3d.set_xlim(-3, 3)
        self.axis3d.set_ylim(-3, 3)
        self.axis3d.set_zlim(-3, 3)
        self.axis3d.set_xlabel('West/East [m]')
        self.axis3d.set_ylabel('South/North [m]')
        self.axis3d.set_zlabel('Down/Up [m]')
        self.axis3d.set_title("Time %.3f s" %(self.t))
        pl.pause(0.001)
        pl.draw()
