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
from gym_aero.envs.helper import Sphere

class PlannerBoxEnv(gym.Env):
    def __init__(self, num_obstacles=10, max_rad=1., length=5, width=5, height=5):
        metadata = {'render.modes': ['human']}
        self.num_obstacles = num_obstacles
        self.max_rad = max_rad
        self.length = length
        self.width = width
        self.height = height
        self.steps = 50
        self.count = 0
        self.action_space = np.zeros((3,))
        self.observation_space = np.zeros((6+int(num_obstacles*4),))
        self.__max_step = 1.5
        self.goal_thresh = 0.1
        self.action_bound = [0, self.__max_step]
        print("Action and Observation spaces initialized")

        # waypoint goals
        self.xyz = np.zeros((3,))
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

        self.col_rad = cfg.params["l"]+cfg.params["prop_radius"]
        
        print("Generating obstacles")
        self.obstacles = self.generate_obstacles()

        # final goal
        print("Generating goal")
        self.goal_xyz = self.generate_goal()
        self.vec_xyz = self.xyz-self.goal_xyz

        # for open_gl animation
        self.init_rendering = False

    # check collision for input point with obstacles
    def check_collision(self, pos):
        cols = [np.linalg.norm(pos-s.get_center()) <= s.get_radius()+self.col_rad for s in self.obstacles]
        return sum(cols) > 0
    
    def generate_obstacles(self):
        # generate obstacles
        obstacles = []
        for i in range(self.num_obstacles):
            collision = True
            while collision:
                obs = Sphere(self.max_rad, self.length, self.width, self.height)
                collision = np.linalg.norm(self.zero-obs.xyz)<= self.col_rad+obs.rad
            obstacles.append(obs)
        return obstacles
    
    def generate_goal(self):
        def gen_coords():
            x = random.uniform(-self.length, self.length)
            y = random.uniform(-self.width, self.width)
            z = random.uniform(-self.height, self.height)
            return  np.array([[x],[y],[z]])
        collision = True
        bounded = False
        while collision and not bounded:
            goal_xyz = gen_coords()
            dist = np.linalg.norm(goal_xyz-self.zero)
            collision = self.check_collision(goal_xyz)
            if dist < self.max_rad:
                bounded = True
        return goal_xyz

    def set_lazy_action(self, arg):
        pass

    def set_lazy_change(self, arg):
        pass

    def terminal(self, xyz):
        mask1 = np.abs(xyz[0]) > self.length
        mask2 = np.abs(xyz[1]) > self.width
        mask3 = np.abs(xyz[2]) > self.height
        if mask1 or mask2 or mask3:
            print("Out of bounds")
            return True
        if np.linalg.norm(self.vec_xyz)<self.goal_thresh:
            print("Goal reached in {} steps".format(self.count))
            return True
        if self.count >= self.steps:
            #print("Goal not reached after {} steps".format(self.count))
            return True
        col = self.check_collision(xyz)
        if col:
            print("Collided")
            return True
        return False

    def get_reward(self, next_state):
        d1 = np.linalg.norm(self.xyz-self.goal_xyz)
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
        collision = self.check_collision(next_state)
        if collision:
            col_rew -= 500.
        d2 = np.linalg.norm(vec_xyz)
        dist_rew = 100.*(d1-d2)
        step_rew = -2.
        self.vec_xyz = vec_xyz
        return dist_rew, guide_rew, cmplt_rew, step_rew, oob_rew, col_rew

    def process_action(self, action):
        action_mag = np.linalg.norm(action)
        if action_mag != 0.:
            if action_mag > self.__max_step:
                return (action*(self.__max_step/action_mag)).reshape((3,1))
            else:
                return action.reshape((3,1))
        else:
            return np.zeros((3,1))

    def step(self, action):
        command = self.process_action(action)
        next_waypoint = self.xyz+command
        self.waypoint_list.append(next_waypoint.copy())
        info = self.get_reward(next_waypoint)
        rew = sum(info)
        position_obs = sum([(next_waypoint-obs.xyz).T.tolist()[0]+[obs.rad] for obs in self.obstacles],[])
        done = self.terminal(next_waypoint)
        next_state = next_waypoint.T.tolist()[0]+position_obs+self.vec_xyz.T.tolist()[0]
        self.xyz = next_waypoint
        self.count += 1
        return next_state, rew, done, {"dist_rew": info[0],
                                        "guide_rew": info[1],
                                        "cmplt_rew": info[2],
                                        "step_rew": info[3],
                                        "oob_rew": info[4]}
        
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
        self.waypoint_list = []
        self.count = 0
        self.obstacles = self.generate_obstacles()
        self.goal_xyz = self.generate_goal()
        xyz = np.zeros((3,1))
        state = xyz.T.tolist()[0]
        position_obs = sum([(xyz-obs.xyz).T.tolist()[0]+[obs.rad] for obs in self.obstacles],[])
        self.xyz = xyz
        self.vec_xyz = (xyz-self.goal_xyz).T.tolist()[0]
        return state+position_obs+self.vec_xyz
    
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
        self.ani.draw_goal(self.zero)
        #for p in self.pts:
        #    self.ani.draw_goal(p[1:], (0.5,0,0.5))
        for i in range(len(self.waypoint_list)):
            wp = self.waypoint_list[i]
            self.ani.draw_goal(wp)
        self.ani.draw_goal(self.goal_xyz, color=(0.5,0,0))
        for s in self.obstacles:
            self.ani.draw_sphere(s.get_center(), s.get_radius())
        self.ani.draw_label("Step: {}".format(self.count), 
            (self.ani.window.width // 2, 20.0))
        self.ani.draw()