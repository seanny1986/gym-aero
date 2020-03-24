from gym_aero.envs import env_base
from math import sin, cos, acos, pi
import gym
import numpy as np
import random

class LandEnv(env_base.AeroEnv):
    """
        Environment wrapper for training low-level flying skills. The aircraft is required to land
        at a random positon. It is required to lower onto the landing pad, i.e the angle
        between the aircraft and pad is 90 deg.
    """
    def __init__(self):
        super(LandEnv, self).__init__()
        self.name = "Land-v0"

        self.goal_xyz = [0, 0, 0]
        self.goal_zeta = [0, 0, 0]
        self.goal_xyz_dot = [0, 0, 0]
        self.goal_pqr = [0, 0, 0]
        
        self.max_dist = 5
        self.pos_thresh = 0.05
        self.ang_thresh = 5.*pi/180.
        self.T = 5
        self.start_radius = 1.5
        self.start_height = 3.
        self._max_episode_steps = int(self.T/self.ctrl_dt)

        self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(20,))

        self.points = [[0, self.moment_arm + 0.05, 0],
                        [0, -(self.moment_arm + 0.05), 0],
                        [self.moment_arm + 0.05, 0, 0],
                        [-(self.moment_arm + 0.05), 0, 0]]

    def reward(self, state, action):
        xyz, sin_zeta, cos_zeta, xyz_dot, pqr = state

        # agent gets a negative reward based on how far away it is from the desired goal state
        dist_rew = -10 * self.curr_dist**2
        #dist_rew += 10 * (self.curr_dist - self.prev_dist)
        
        att_rew = -10 * (self.curr_att_sin**2 + self.curr_att_cos**2)
        #att_rew += 10*((self.prev_att_sin-self.curr_att_sin)+(self.prev_att_cos-self.curr_att_cos))
        
        vel_rew = -10*self.curr_vel**2
        #vel_rew += 10*(self.prev_vel-self.curr_vel)
        
        ang_rew = -10*self.curr_ang**2
        #ang_rew += 10*(self.prev_ang-self.curr_ang)

        ctrl_rew = 0.
        ctrl_rew -= sum([((a-self.hov_rpm)/self.max_rpm)**2 for a in action])
        ctrl_rew -= sum([((a-pa)/self.max_rpm)**2 for a, pa in zip(action, self.prev_action)])
        ctrl_rew -= 10*sum([(u-v)**2 for u, v in zip(xyz_dot, self.prev_xyz_dot)])
        ctrl_rew -= 10*sum([(p-q)**2 for p, q in zip(pqr, self.prev_pqr)])

        time_rew = 0.

        uvw = self.inertial_to_body(xyz_dot)
        vrs_rew = -500 if uvw[2] < -2.  else 0
        crash_rew = -500 if self.crashed(xyz) else 0
        
        #complete_rew = 500 if att_rew/10. < self.ang_thresh else 0
        #complete_rew = complete_rew + 500 if dist_rew/100. < self.pos_thresh else complete_rew
        complete_rew = 0

        total_reward = dist_rew+att_rew+vel_rew+ang_rew+ctrl_rew+vrs_rew+crash_rew+complete_rew+time_rew
        return total_reward, {"dist_rew": dist_rew,
                                "att_rew": att_rew,
                                "vel_rew": vel_rew,
                                "ang_rew": ang_rew,
                                "ctrl_rew": ctrl_rew,
                                "vrs_rew": vrs_rew,
                                "crash_rew": crash_rew,
                                "complete_rew": complete_rew,
                                "time_rew": time_rew}
    
    def crashed(self, xyz):
        rotated = [self.inertial_to_body(p) for p in self.points]
        translated = [[r + x for r, x in zip(rot, xyz)] for rot in rotated]
        crashed = sum([t[-1] > 0 for t in translated])
        return crashed > 0

    def terminal(self, state):
        xyz, zeta, uvw, pqr = state
        # todo: implement radius collision check
        if self.curr_dist >= self.max_dist:
            #print("Max dist exceeded")
            return True
        elif self.t*self.ctrl_dt > self.T:
            #print("Time limit exceeded") 
            return True
        elif uvw[2] > 2.5:
            #print("VRS")
            return True
        elif self.crashed(xyz):
            #print("Crashed")
            return True
        else: 
            return False

    def generate_random_state(self):
        mag = 10.
        while mag > self.start_radius:
            xy = [random.uniform(-self.start_radius, self.start_radius) for _ in range(2)]
            mag = sum([x**2 for x in xy])**0.5
        xyz = xy+[-self.start_height]
        return xyz, [0., 0., 0.], [0., 0., 0.], [0., 0., 0.]

    def reset(self):
        xyz, zeta, pqr, uvw = self.generate_random_state()
        state = super(LandEnv, self).reset_to_custom_state(xyz, zeta, uvw, pqr, self.hov_rpm_)
        xyz, sin_zeta, cos_zeta, uvw, pqr, normalized_rpm = state
        xyz_dot = self.get_xyz_dot()
        self.set_current_dists((xyz, sin_zeta, cos_zeta, xyz_dot, pqr), self.hov_rpm_)
        obs = self.get_state_obs((xyz, sin_zeta, cos_zeta, xyz_dot, pqr), self.hov_rpm_, normalized_rpm)
        self.set_prev_dists((xyz, sin_zeta, cos_zeta, xyz_dot, pqr), self.hov_rpm_)
        return obs
    
    def render(self, mode='human', video=False, close=False):
        super(LandEnv, self).render(mode=mode, close=close)
        self.ani.draw_goal(self.goal_xyz)
        self.ani.draw()
        if video: self.ani.save_frame(self.name)
        if close:
            self.ani.close_window()
            self.init_rendering = False