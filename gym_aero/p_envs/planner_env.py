import numpy as np
import random
from math import pi, sin, cos, sqrt
import gym
from gym import error, spaces, utils
from gym.utils import seeding
from gym_aero.envs import env_base
import simulation.animation_gl as ani_gl
from operator import itemgetter

class PlannerEnv(gym.Env):
    def __init__(self, max_rad=5., length=5, width=5, height=5):
        metadata = {'render.modes': ['human']}
        self.max_rad = max_rad
        self.length = length
        self.width = width
        self.height = height
        self.goal_thresh = 0.1
        self.steps = 25
        self.count = 0
        self.action_space = np.zeros((3,))
        self.observation_space = np.zeros((6,))
        self.__max_step = 1.5
        self.__clip_val = sqrt((self.__max_step**2)/3.)
        self.action_bound = [0, self.__clip_val]
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
        self.filtered_waypoints = []
        self.waypoints_reached = 0
        
        # final goal
        print("Generating goal")
        self.goal_xyz = self.generate_goal()
        self.vec_xyz = self.xyz-self.goal_xyz

        # for open_gl animation
        self.init_rendering = False
    
    def filter_list(self):
        ls = [self.zero]+self.waypoint_list+[self.goal_xyz]
        filtered_waypoints = []
        filtered_waypoints.append(self.goal_xyz)
        
        # pass one: get dist from origin
        distances = []
        for p in ls:
            distances.append(np.linalg.norm(p-self.zero))
        
        # sort list in descending order by distance from origin
        sort = sorted(zip(ls, distances), key=itemgetter(1))
        
        # start at at goal
        start = next(i for i in range(len(sort)) if sort[i][0] is self.goal_xyz)
        
        # all remaining elements are now closer to the origin than the goal
        i = start
        while i >= 0:
            j = i-1
            best = 0
            best_idx = j
            while j >= 0:
                dist = np.linalg.norm(sort[j][0]-sort[i][0])
                if dist < self.__max_step:
                    if dist > best:
                        best_idx = j
                        best = dist
                j -= 1
            filtered_waypoints.append(sort[best_idx][0])
            i = best_idx
        #print(filtered_waypoints)
        self.filtered_waypoints = filtered_waypoints[::-1]

    def terminal(self, pos):
        xyz = pos      
        mask1 = np.abs(xyz[0]) > self.length
        mask2 = np.abs(xyz[1]) > self.width
        mask3 = np.abs(xyz[2]) > self.height
        if mask1 or mask2 or mask3:
            return True
        if np.linalg.norm(self.vec_xyz)<self.goal_thresh:
            print("Goal reached in {} steps".format(self.count))
            return True
        if self.count >= self.steps:
            #print("Goal not reached after {} steps".format(self.count))
            return True
        return False

    def get_reward(self, next_state):
        d1 = np.linalg.norm(self.xyz-self.goal_xyz)
        vec_xyz = next_state-self.goal_xyz
        guide_rew = 1./np.linalg.norm(vec_xyz)**2
        cmplt_rew = 0.
        if np.linalg.norm(vec_xyz) <= self.goal_thresh:
            cmplt_rew = 100.
        mask1 = np.abs(next_state[0]) > self.length
        mask2 = np.abs(next_state[1]) > self.width
        mask3 = np.abs(next_state[2]) > self.height
        oob_rew = 0.
        if mask1 or mask2 or mask3:
            oob_rew = -100.
        d2 = np.linalg.norm(vec_xyz)
        dist_rew = (d1-d2)
        step_rew = -1.
        self.vec_xyz = vec_xyz
        return dist_rew, guide_rew, cmplt_rew, step_rew, oob_rew

    def get_goal(self):
        return self.goal_xyz

    def generate_goal(self):
        def gen_coords():
            x = random.uniform(-self.length, self.length)
            y = random.uniform(-self.width, self.width)
            z = random.uniform(-self.height, self.height)
            return  np.array([[x],[y],[z]])
        dist = 100.
        while dist > self.max_rad:
            goal_xyz = gen_coords()
            dist = np.linalg.norm(goal_xyz-np.zeros((3,1)))
        return goal_xyz

    def step(self, action):
        command = np.clip(action, -self.__clip_val, self.__clip_val).reshape((3,1))
        next_state = self.xyz+command
        self.waypoint_list.append(next_state.copy())
        info = self.get_reward(next_state)
        rew = sum(info)
        self.xyz = next_state
        done = self.terminal(next_state)
        next_state = next_state.T.tolist()[0]+self.vec_xyz.T.tolist()[0]
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
        self.goal_xyz = self.generate_goal()
        xyz = np.zeros((3,1))
        state = xyz.T.tolist()[0]
        self.vec_xyz = (xyz-self.goal_xyz).T.tolist()[0]
        self.xyz = xyz
        return state+self.vec_xyz

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
        for i in range(len(self.waypoint_list)):
            wp = self.waypoint_list[i]
            self.ani.draw_goal(wp)
        self.ani.draw_goal(self.goal_xyz, color=(0.5,0,0))
        self.ani.draw_label("Time: {0:.2f}".format(self.count), 
            (self.ani.window.width // 2, 20.0))
        self.ani.draw()