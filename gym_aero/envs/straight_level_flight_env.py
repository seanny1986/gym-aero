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


"""
    Environment wrapper for a hover task. The goal of this task is for the agent to climb from [0, 0, 0]^T
    to [0, 0, 1.5]^T, and to remain at that altitude until the the episode terminates at T=15s.
"""

class StraightLevelFlightEnv(gym.Env):
    def __init__(self):
        metadata = {'render.modes': ['human']}

        # environment parameters
        self.goal_xyz = np.array([[0.],
                                [0.],
                                [0.]])
        self.goal_zeta_sin = np.sin(np.array([[0.],
                                            [0.],
                                            [0.]]))
        self.goal_zeta_cos = np.cos(np.array([[0.],
                                            [0.],
                                            [0.]]))
        self.goal_uvw = np.array([[0.],
                                [0.],
                                [0.]])
        self.goal_pqr = np.array([[0.],
                                [0.],
                                [0.]])

        self.goal_alt = 0;
        self.goal_sway = 0;
        self.goal_dir = np.array([[1.0],[0.0],[0.0]]);
        self.vec_to_path = np.array([[0.0],[0.0],[0.0]]);
        self.alt_deviance = 1.0;
        self.max_alt_deviance = 2.5;
        self.sway_deviance = 1.0;
        self.max_sway_deviance = 2.5;
        self.on_path = True;
        self.too_far_from_path = False;

        self.t = 0
        self.T = 20
        self.action_space = np.zeros((4,))
        self.observation_space = np.zeros((25,))

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
        self.bandwidth = 25.

        self.iris.set_state(self.goal_xyz, np.arcsin(self.goal_zeta_sin), self.goal_uvw, self.goal_pqr)
        xyz, zeta, uvw, pqr = self.iris.get_state()

        self.fig = None
        self.axis3d = None

    def get_goal(self):
        return self.goal_xyz

    def reward(self, state, action):
        xyz, zeta, uvw, pqr = state

        x,y,z = xyz.T[0];
        self.vel = self.iris.get_inertial_velocity();
        #Calculate the velocity which the agent is travelling in the
        #plane which is orthoganl to the goal path
        self.offpath_vel = self.vel;
        self.offpath_vel[0][0] = 0.0;

        #Calculate vector from agent back to path
        self.vec_to_path = -xyz;
        self.vec_to_path[0][0] = 0.0;

        alt_diff = abs(z - self.goal_alt);
        sway_diff = abs(y - self.goal_sway);

        dist_rew = 0;
        time_rew = 0;
        on_path_rew = 0;
        speed_cost = 0;
        ang_speed_cost = 0;
        ctrl_rew = 0;

        atAltitude = alt_diff < self.alt_deviance;
        onPathHoriz = sway_diff < self.sway_deviance;

        pastMaxAltitude = alt_diff > self.max_alt_deviance;
        pastMaxPathHoriz = sway_diff > self.max_sway_deviance;

        #Check if the agent is following the path
        self.on_path = atAltitude and onPathHoriz and x >= 0;
        #Check the agent isn't too far away from the path
        self.too_far_from_path = pastMaxAltitude or pastMaxPathHoriz;        

        #Agent can only receive some rewards if it is following
        #the path within reasonable margin of error
        if(self.on_path):
            #Receive reward for following path
            on_path_rew = 1;
            #Receive reward for travelling along the path
            dist_rew = (x - self.x_pos) / self.ctrl_dt;
            #Receive reward for surviving
            time_rew = 1.0;

        #Cost for moving in the plane opposite to the line
        speed_cost = -1.0 * np.linalg.norm(self.offpath_vel);
        #Cost for spinning too quickly
        ang_speed_cost = -1.0 * np.linalg.norm(pqr);

        mask1 = zeta[:] > (2*pi)/3;
        mask2 = zeta[:] < (2*-pi)/3;
        #Cost for spinning too far in any axis
        ang_cost = -.5 * (sum(mask1) + sum(mask2))[0];

        #Agent gets a negative reward for excessive action inputs
        ctrl_rew = -np.sum(((action/self.action_bound[1])**2));

        #Store last x position
        self.x_pos = x;

        total_rew = dist_rew, time_rew, ang_cost, ctrl_rew, on_path_rew, speed_cost, ang_speed_cost;

        return total_rew;

    def terminal(self, pos):
        xyz, zeta = pos
        #Terminate simulation when agent is at a 90 degree angle
        mask1 = zeta[:] > pi/2
        mask2 = zeta[:] < -pi/2
        #Terminate simulation when agent has left container in y/z dimension
        mask3 = abs(xyz.T[0][1]) > 3 or abs(xyz.T[0][2]) > 3;

        #Terminate simulation if agent is too far from path
        if(self.too_far_from_path):
            return True;

        if np.sum(mask1) > 0 or np.sum(mask2) > 0 or mask3:
            return True
        elif self.t >= self.T:
            #Simulation completed successfully
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
        
        done = self.terminal((xyz, zeta))
        info = self.reward((xyz, zeta, uvw, pqr), action)
        next_state = xyz.T.tolist()[0]+sin_zeta.T.tolist()[0]+cos_zeta.T.tolist()[0]+uvw.T.tolist()[0]+pqr.T.tolist()[0]
        reward = sum(info)
        goals = self.goal_dir.T.tolist()[0] + self.vec_to_path.T.tolist()[0];
        next_state = next_state+a+goals
        return next_state, reward, done, info

    def reset(self):
        self.t = 0.
        self.iris.set_state(self.goal_xyz, np.sin(self.goal_zeta_sin), self.goal_uvw, self.goal_pqr)
        self.iris.set_rpm(np.array(self.trim))
        xyz, zeta, uvw, pqr = self.iris.get_state()
        sin_zeta = np.sin(zeta)
        cos_zeta = np.cos(zeta)
       
        a = [x/self.action_bound[1] for x in self.trim]
        goals = self.goal_dir.T.tolist()[0] + self.vec_to_path.T.tolist()[0];
        state = xyz.T.tolist()[0]+sin_zeta.T.tolist()[0]+cos_zeta.T.tolist()[0]+uvw.T.tolist()[0]+pqr.T.tolist()[0]+a+goals

        self.x_pos = 0.0;

        return state
    
    def render(self, mode='human', close=False):
        if self.fig is None:
            pl.close("all")
            pl.ion()
            self.fig = pl.figure("Straight Level Flight")
            self.axis3d = self.fig.add_subplot(111, projection='3d')
            self.vis = ani.Visualization(self.iris, 6, quaternion=True)
        pl.figure("Straight Level Flight")
        self.axis3d.cla()
        self.vis.draw3d_quat(self.axis3d)
        self.vis.draw_goal(self.axis3d, self.goal_xyz)
        #Move environment with agent
        self.axis3d.set_xlim(-3 + self.x_pos, 3 + self.x_pos)
        self.axis3d.set_ylim(-3, 3)
        self.axis3d.set_zlim(-3, 3)
        self.axis3d.set_xlabel('West/East [m]')
        self.axis3d.set_ylabel('South/North [m]')
        self.axis3d.set_zlabel('Down/Up [m]')
        self.axis3d.set_title("Time %.3f s" %(self.t))

        xyz, _, _, _ = self.iris.get_state()

        start_line_pos = [max(0.0, self.x_pos - 3.0), 0, 0];
        end_line_pos = [3 + self.x_pos, 0, 0];
        color = [0,1,0];
        if(not self.on_path):
            color = [1,0,0];
        self.vis.draw_line(self.axis3d, start_line_pos, end_line_pos, color=color);

        self.vis.draw_line(self.axis3d, xyz.T.tolist()[0], (xyz + self.vec_to_path).T.tolist()[0], color=[1,0,1]);

        pl.pause(0.0001)
        pl.draw()


