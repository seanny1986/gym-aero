import simulation.quadrotor3 as quad
import simulation.config as cfg
import numpy as np
import random
from math import pi, sin, cos, sqrt
import gym
from gym import error, spaces, utils
from gym.utils import seeding
from gym_aero.envs.Box_world_helper import *
import simulation.animation_gl as ani_gl

class PlannerEnv(gym.Env):
    def __init__(self, length=5, width=5, height=5):
        metadata = {'render.modes': ['human']}
        self.length = length
        self.width = width
        self.height = height
        self.goal_thresh = 0.075
        self.safety_fac = 0.75
        self.steps = 25
        self.count = 0
        self.action_space = np.zeros((3,))
        self.observation_space = np.zeros((6,))
        self.__max_step = 1.5
        self.__clip_val = sqrt((self.__max_step**2)/3.)
        self.action_bound = [0, self.__clip_val]
        print("Action and Observation spaces initialized")

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
        self.zero = np.zeros((3,1))
        self.datum = np.zeros((3,1))
        self.waypoint_list = []
        self.waypoints_reached = 0
        
        # final goal
        print("Generating goal")
        self.goal_xyz = self.generate_goal()

        # for open_gl animation
        self.init_rendering = False

    def generate_goal(self):
        def gen_coords():
            x = random.uniform(-self.length, self.length)
            y = random.uniform(-self.width, self.width)
            z = random.uniform(-self.height, self.height)
            return  np.array([[x],[y],[z]])
        goal_xyz = gen_coords()
        return goal_xyz

    def planner_step(self, xyz, action):
        command = torch.clamp(action, -self.__clip_val, self.__clip_val)
        next_state = xyz.reshape((3,1))+command
        d1 = np.linalg.norm(xyz-self.goal_xyz)
        vec = next_state-self.goal_xyz
        d2 = np.linalg.norm(vec)
        dist_rew = d1-d2
        terminate = False
        cmplt_rew = 0.
        if np.linalg.norm(next_state-self.goal_xyz)<self.goal_thresh:
            cmplt_rew += 100.
            self.planner_lock = True
            terminate = True
        if self.count >= self.steps:
            terminate = True
        rew = dist_rew
        next_state = next_state.T.tolist()[0]+vec.T.tolist()[0]
        self.count += 1
        return next_state, rew, terminate, {"dist_rew": dist_rew}
        
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
        self.count = 0
        self.goal_xyz = self.generate_goal()
        xyz = np.zeros((3,1))
        state = xyz.T.tolist()[0]
        vec = (xyz-self.goal_xyz).T.tolist()[0]
        return state+vec
    
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
            self.ani.draw_goal(wp)
        self.ani.draw_goal(self.goal_xyz, color=(0.5,0,0))
        self.ani.draw_label("Time: {0:.2f}".format(self.count), 
            (self.ani.window.width // 2, 20.0))
        self.ani.draw()