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
import math


class LandEnv(gym.Env):
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
                                [-3]])
        self.spawn = np.array([[random.uniform(-3,3)],
                                [random.uniform(-3,3)],
                                [3.]])

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
        print('NEW!!')
        #self.vel_z = iris.get_inertial_velocity()[0]

        self.goal_decent_speed = -0.25
        self.decent_speed = self.iris.get_inertial_velocity()[2][0] - self.goal_decent_speed

        self.dist_norm = np.linalg.norm(self.vec_xyz)
        self.att_norm_sin = np.linalg.norm(self.vec_zeta_sin)
        self.att_norm_cos = np.linalg.norm(self.vec_zeta_cos)
        self.vel_norm = np.linalg.norm(self.vec_uvw)
        self.ang_norm = np.linalg.norm(self.vec_pqr)
        #self.landing_angle = (180/np.pi)*abs(np.arcsin(temp_O/dist_hat))[0])-90

        self.distance_decrease = False

        self.fig = None
        self.axis3d = None

    def reward(self, state, action,terminal):
        xyz, zeta, uvw, pqr = state
        s_zeta = np.sin(zeta)
        c_zeta = np.cos(zeta)
        curr_dist = xyz-self.goal_xyz
        curr_att_sin = s_zeta-self.goal_zeta_sin
        curr_att_cos = c_zeta-self.goal_zeta_cos
        curr_vel = uvw-self.goal_uvw
        curr_ang = pqr-self.goal_pqr
        #print(uvw)

        # magnitude of the distance from the goal
        dist_hat = np.linalg.norm(curr_dist)
        att_hat_sin = np.linalg.norm(curr_att_sin)
        att_hat_cos = np.linalg.norm(curr_att_cos)
        vel_hat = np.linalg.norm(curr_vel)
        ang_hat = np.linalg.norm(curr_ang)

        # agent gets a negative reward based on how far away it is from the desired goal state
        dist_rew = 100*(self.dist_norm-dist_hat) #+ self.get_sigmoid_val(0,dist_hat)
        #dist_rew = -abs(xyz[2][0]-self.goal_xyz[2][0])
        ##10 is slow
        #dist_rew = 5*self.get_sigmoid_val(0,dist_hat)

        # above_pad = ((xyz[0]-self.goal_xyz[0])**2 + (xyz[1]-self.goal_xyz[1])**2)**0.5
        # if(above_pad[0] < 0.5):
        #     dist_rew += 1.5*dist_rew

        """ maybe put this in, but might make it unstable

            if dist_rew < 0.4:
                dist_rew = 2*dist_rew
        """

        ## Difference in distance with sigmoid migt better option


        if(dist_rew > 0):
            self.distance_decrease = True
        else:
            self.distance_decrease = False

        ##Land v0 and land v3

        roll = zeta[0]*180/np.pi
        pitch = zeta[1]*180/np.pi
        yaw = zeta[2]*180/np.pi

        #att_rew = 1*(self.get_sigmoid_val(0,2*roll) + self.get_sigmoid_val(0,2*pitch))+ self.get_sigmoid_val(0,2*yaw)


        #att_rew = -0.1*(abs(roll)+abs(pitch)+abs(yaw))[0]
        ##V2 USES BELLOW ATT_REW and no 90 deg

        att_rew = 0*((self.att_norm_sin-att_hat_sin)+(self.att_norm_cos-att_hat_cos))
        #att_rew = 0*(self.get_sigmoid_val(0,att_hat_sin) +  self.get_sigmoid_val(0,att_hat_cos))-2


        #print(att_rew,roll,pitch,yaw)
        land_speed_rew = 0#-10*abs(self.iris.get_inertial_velocity())[0][0]


        temp_O = xyz[2] - self.goal_xyz[2]


        x_dot = self.iris.get_inertial_velocity()
        #print(x_dot)
        land_angle_rew =5*self.get_sigmoid_val(-0.5*10,10*x_dot[2][0])
        #print(land_angle_rew,x_dot[2][0])

        vel_rew = 10*(self.vel_norm-vel_hat)
        ang_rew = 10*(self.ang_norm-ang_hat) #THIS WAS 1, with no V3

        time_rew = -10000 if (terminal and self.t < self.T) else 0;




        decent_hat = self.iris.get_inertial_velocity()[2][0] - self.goal_decent_speed
        #land_angle_rew = 100*(self.decent_speed- decent_hat)
        self.decent_speed = decent_hat


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


        ctrl_rew = -np.sum(((action/self.action_bound[1])**2))

        # agent gets a positive reward for time spent in flight
        fall_vel_rew = 0
        time_rew = 0#.1*self.t






        #print("D: ",dist_rew,"AT: ",att_rew,"LS: ",land_angle_rew,"LX ",land_speed_rew, "V ",vel_rew,"A ",ang_rew, "Ti ",time_rew )
        # print(roll,pitch,yaw)
        return dist_rew, att_rew, vel_rew, ang_rew, ctrl_rew, time_rew, land_angle_rew,land_speed_rew


    ##e val is the expected value of the sigmoid func
    def get_sigmoid_val(self,e_val,val):
        sig = 1/(1+math.exp(-val+e_val))
        dir_sig = 4*sig*(1-sig)
        return dir_sig


    def terminal(self, pos,): ##//TODO:: PASS UvW
        xyz, zeta, uvw = pos
        #print(uvw.T.tolist())
        mask1  = zeta[0:2] > pi/2
        mask2 = zeta[0:2] < -pi/2
        mask3 = False#self.dist_norm > 5
        orign = np.array([[0.],
                                [0.],
                                [0.]])
        orign_dist = np.linalg.norm(xyz - orign)
        goal_dist = np.linalg.norm(xyz - self.goal_xyz)
        #print(uvw[2])

        if(uvw[2][0] < -2.):
            #print("Aircraft Lost")
            return True
        elif np.sum(mask1) > 0 or np.sum(mask2) > 0 or np.sum(mask3) > 0:
            return True

        elif xyz[2] < -3.5:
            return True
        # elif(orign_dist > 8.):False
        #     print("Out of Env")
        #     return True
        ##Ends it when the goal is near
        elif goal_dist < 0.2:
            print("Goal Achieved!")
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
        #print(action)
        for _ in self.steps:
            xyz, zeta, uvw, pqr = self.iris.step(self.trim_np+action*self.bandwidth)
        self.t += self.ctrl_dt

        sin_zeta = np.sin(zeta)
        cos_zeta = np.cos(zeta)
        a = (action/self.action_bound[1]).tolist()
        next_state = xyz.T.tolist()[0]+sin_zeta.T.tolist()[0]+cos_zeta.T.tolist()[0]+uvw.T.tolist()[0]+pqr.T.tolist()[0]

        done = self.terminal((xyz, zeta,uvw))
        info = self.reward((xyz, zeta, uvw, pqr), action,done)
        reward = sum(info)
        #print("REWARD ",reward)
        goals = self.vec_xyz.T.tolist()[0]+self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]+self.vec_uvw.T.tolist()[0]+self.vec_pqr.T.tolist()[0]
        next_state = next_state+a+goals
        return next_state, reward, done, info


    def get_goal(self):
        return self.goal_xyz

    def reset(self):
        #print('------NEW RESET------')
        self.goal_achieved = False
        self.t = 0.

        self.goal_xyz = np.array([[random.uniform(-3,3)],
                                [random.uniform(-3,3)],
                                [-3]])
        self.spawn = np.array([[random.uniform(-3,3)],
                                [random.uniform(-3,3)],
                                [3.]])

        self.iris.set_state(self.spawn, np.arcsin(self.goal_zeta_sin), self.goal_uvw, self.goal_pqr)
        self.iris.set_rpm(np.array(self.trim))
        self.decent_speed = self.iris.get_inertial_velocity()[2][0] - self.goal_decent_speed
        xyz, zeta, uvw, pqr = self.iris.get_state()

        ##print(xyz.T.tolist(),self.spawn.T.tolist())

        sin_zeta = np.sin(zeta)
        cos_zeta = np.cos(zeta)
        self.vec_xyz = xyz-self.goal_xyz
        self.vec_zeta_sin = sin_zeta
        self.vec_zeta_cos = cos_zeta
        self.dist_norm = np.linalg.norm(self.vec_xyz)
        self.vec_uvw = uvw
        self.vec_pqr = pqr
        a = (self.trim_np/self.action_bound[1]).tolist()
        goals = self.vec_xyz.T.tolist()[0]+self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]+self.vec_uvw.T.tolist()[0]+self.vec_pqr.T.tolist()[0]
        state = xyz.T.tolist()[0]+sin_zeta.T.tolist()[0]+cos_zeta.T.tolist()[0]+uvw.T.tolist()[0]+pqr.T.tolist()[0]+a+goals
        return state


    def get_y(self,x):
        #temp =  (-(x+30)*(x-30))
        #return (temp/900) * 3
        temp = -x**2
        return temp
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
        cir = pl.Circle((self.goal_xyz[0],self.goal_xyz[1]), 0.5)
        self.axis3d.add_patch(cir)
        art3d.pathpatch_2d_to_3d(cir, z=self.goal_xyz[2], zdir="z")

        xyz, zeta, uvw, pqr = self.iris.get_state()

        if self.distance_decrease:
            self.axis3d.plot([self.goal_xyz[0][0],xyz[0][0]],[self.goal_xyz[1][0],xyz[1][0]],[self.goal_xyz[2][0],xyz[2][0]],c='g')
        else:
            self.axis3d.plot([self.goal_xyz[0][0],xyz[0][0]],[self.goal_xyz[1][0],xyz[1][0]],[self.goal_xyz[2][0],xyz[2][0]],c='r')


        self.axis3d.plot([xyz[0][0],xyz[0][0]],[xyz[1][0],xyz[1][0]],[xyz[2][0],xyz[2][0]+uvw[2]],c='b')
        self.axis3d.plot([xyz[0][0],xyz[0][0]],[xyz[1][0],xyz[1][0]],[xyz[2][0],xyz[2][0]+self.iris.get_inertial_velocity()[2][0]],c='r')



        self.axis3d.set_xlim(-3, 3)
        self.axis3d.set_ylim(-3, 3)
        self.axis3d.set_zlim(-3, 3)
        self.axis3d.set_xlabel('West/East [m]')
        self.axis3d.set_ylabel('South/North [m]')
        self.axis3d.set_zlabel('Down/Up [m]')
        self.axis3d.set_title("Time %.3f s" %(self.t))
        pl.pause(0.001)
        pl.draw()
