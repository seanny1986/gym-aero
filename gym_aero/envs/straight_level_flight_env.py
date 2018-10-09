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

def exp_moving_avg(avg, sample, alpha):
    return ((1.0 - alpha) * avg) + (alpha * sample);

def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x));


class PID:
    def __init__(self, P=2.0, I=0.0, D=1.0, Derivator=0, Integrator=0, Integrator_max=500, Integrator_min=-500):
        self.Kp=P
        self.Ki=I
        self.Kd=D
        self.Derivator=Derivator
        self.Integrator=Integrator
        self.Integrator_max=Integrator_max
        self.Integrator_min=Integrator_min

        self.set_point=0.0
        self.error=0.0

    def update(self,current_value):
        """
        Calculate PID output value for given reference input and feedback
        """

        self.error = self.set_point - current_value;
        self.P_value = self.Kp * self.error;
        self.D_value = self.Kd * ( self.error - self.Derivator);
        self.Derivator = self.error;
        self.Integrator = self.Integrator + self.error;

        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max;
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min;

        self.I_value = self.Integrator * self.Ki;

        return self.P_value + self.I_value + self.D_value;

    def setPoint(self,set_point):
        """
        Initilize the setpoint of PID
        """
        self.set_point = set_point
        self.Integrator=0
        self.Derivator=0


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
        self.alt_deviance = 0.7;
        self.alt_error = 0.25;
        self.sway_deviance = 0.7;
        self.sway_error = 0.25;
        self.alpha = 0.3;
        self.on_path = True;

        self.t = 0
        self.T = 20
        self.action_space = np.zeros((4,))
        self.observation_space = np.zeros((25,))

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
        self.bandwidth = 25.

        self.iris.set_state(self.goal_xyz, np.arcsin(self.goal_zeta_sin), self.goal_uvw, self.goal_pqr)
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

        self.fig = None
        self.axis3d = None

    def get_goal(self):
        return self.goal_xyz

    def reward(self, state, action):
        xyz, zeta, uvw, pqr = state

        x,y,z = xyz.T[0];
        self.vel = self.iris.get_inertial_velocity();

        self.vec_to_path = -xyz;
        self.vec_to_path[0][0] = 0.0;

        alt_diff = abs(z - self.goal_alt);
        sway_diff = abs(y - self.goal_sway);

        alt_weighted_diff = abs(z - self.goal_alt);
        sway_weighted_diff = abs(y - self.goal_sway);

        allowable_alt_diff = self.alt_deviance * self.alt_error;
        allowable_sway_diff = self.sway_deviance * self.sway_error;

        self.sway_diff_avg = exp_moving_avg(self.sway_diff_avg, sway_diff, self.alpha);
        self.alt_diff_avg = exp_moving_avg(self.alt_diff_avg, alt_diff, self.alpha);

        dist_rew = 0;
        alt_rew = 0;
        time_rew = 0;
        horiz_rew = 0;

        horiz_rew_pos = 0;
        horiz_rew_neg = 0;
        alt_rew_pos = 0;
        alt_rew_neg = 0;
        ang_rew = 0;
        top_rew = 0;
        bottom_rew = 0;
        left_rew = 0;
        right_rew = 0;

        atAltitude = alt_diff < self.alt_deviance;
        onPathHoriz = sway_diff < self.sway_deviance;

        self.on_path = atAltitude and onPathHoriz;

        

        if(self.on_path):

            max_dist_rew = 25.0;

            if(x >= -0.1):
                dist_rew = 25.0 * x;
            else:
                dist_rew = -1000.0;
            
            time_rew = 80.0 * self.t;
            # horiz_rew = (-self.sway_diff_avg + 0.5) * 10.0;
            # alt_rew   = (self.alt_diff_avg + 0.5)   * 10.0;

            horiz_pid = self.horiz_pid.update(y - self.goal_sway);
            alt_pid = self.vert_pid.update(z - self.goal_alt);

            if(horiz_pid > 0):
                left_rew = -10 * abs(horiz_pid);
            else:
                right_rew = -10 * abs(horiz_pid);

            if(alt_pid < 0):
                top_rew = -5 * abs(alt_pid);
            else:
                bottom_rew = -10 * abs(alt_pid);


            horiz_rew = -10.0 * horiz_pid;
            alt_rew   = -10.0 * alt_pid;

                # print("%f, %f" %(top_rew, bottom_rew));

            # alt_rew = 10.0 * (-alt_diff + 0.6);
            # horiz_rew = 10.0 * (-alt_diff + 0.6);

        mask1 = zeta[:] > (2*pi)/3;
        mask2 = zeta[:] < (2*-pi)/3;
        ang_rew = -25.0 * (sum(mask1) + sum(mask2))[0];

        self.x_pos = x;

        total_rew =  dist_rew, top_rew, bottom_rew, right_rew, left_rew, time_rew, ang_rew
        # if(int(self.t / self.ctrl_dt) % 5 == 0):
        #     print("REW: " + str(sum(total_rew)));

        return total_rew;

    def terminal(self, pos):
        xyz, zeta = pos
        mask1 = zeta[:] > pi/2
        mask2 = zeta[:] < -pi/2
        # mask1 = 0.0;
        # mask2 = 0.0;
        mask3 = abs(xyz.T[0][1]) > 3 or abs(xyz.T[0][2]) > 3;

        if(not self.on_path):
            return True;

        if np.sum(mask1) > 0 or np.sum(mask2) > 0 or mask3:
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
        done = self.terminal((xyz, zeta))
        info = self.reward((xyz, zeta, uvw, pqr), action)
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
        self.vec_xyz = xyz-self.goal_xyz
        self.vec_zeta_sin = sin_zeta-self.goal_zeta_sin
        self.vec_zeta_cos = cos_zeta-self.goal_zeta_cos
        self.vec_uvw = uvw-self.goal_uvw
        self.vec_pqr = pqr-self.goal_pqr
        self.dist_norm = np.linalg.norm(self.vec_xyz)
        self.att_norm_sin = np.linalg.norm(self.vec_zeta_sin)
        self.att_norm_cos = np.linalg.norm(self.vec_zeta_cos)
        self.vel_norm = np.linalg.norm(self.vec_uvw)
        self.ang_norm = np.linalg.norm(self.vec_pqr)

        self.horiz_pid = PID(P=0.5, I=0.2, D=0.1);
        self.vert_pid  = PID(P=0.5, I=0.2, D=0.1);
        self.horiz_pid.setPoint(self.goal_sway);
        self.vert_pid.setPoint(self.goal_alt)

        a = [x/self.action_bound[1] for x in self.trim]
        goals = self.goal_dir.T.tolist()[0] + self.vec_to_path.T.tolist()[0];
        state = xyz.T.tolist()[0]+sin_zeta.T.tolist()[0]+cos_zeta.T.tolist()[0]+uvw.T.tolist()[0]+pqr.T.tolist()[0]+a+goals

        self.sway_diff_avg = 0;
        self.alt_diff_avg = 0;
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


