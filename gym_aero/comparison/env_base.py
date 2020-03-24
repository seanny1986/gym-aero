import numpy as np

import random
from math import pi, sin, cos, tanh, exp, sqrt, acos

import gym
from gym import error, spaces, utils
from gym.utils import seeding
from old_simulation import animation_gl as ani_gl
import time
import os
from old_simulation import quadrotor3 as quad
from old_simulation import config as cfg

"""
Defines the base class for environments in gym-aero.
Aircraft is modelled in a NED axis system.
-- Sean Morrison, 2019
"""

class AeroEnv(gym.Env):
    def __init__(self):
        super(AeroEnv, self).__init__()
        self.path = os.path.expanduser("~")        
        self.params = cfg.params
        self.iris = quad.Quadrotor(self.params)
        #iris.kd = 0.
        #iris.km = 0.
        self.sim_dt = self.iris.dt
        self.ctrl_dt = 0.05
        self.sim_steps = int(self.ctrl_dt/self.sim_dt)

        self.init_rendering = False

        self.ac_mass = self.iris.mass
        self.sim_gravity = self.iris.g
        self.torque_coeff = self.iris.kq
        self.thrust_coeff = self.iris.kt
        self.moment_arm = self.iris.l

        self.T = None
        self.t = 0

        self.max_omega = sqrt(self.ac_mass * self.sim_gravity / 2 / self.thrust_coeff)
        self.max_rpm = self.omega_to_rpm(self.max_omega)
        self.hov_omega = sqrt(self.ac_mass * self.sim_gravity / 4 / self.thrust_coeff)
        self.hov_rpm = self.omega_to_rpm(self.hov_omega)
        self.hov_rpm_ = [self.hov_rpm, self.hov_rpm, self.hov_rpm, self.hov_rpm]
        self.hov_omega_ = [self.rpm_to_omega(rpm) for rpm in self.hov_rpm_]
        self.action_bandwidth = 35
        
        self.max_thrust =  self.thrust_coeff * self.max_omega**2
        self.max_u1 = 4 * self.max_thrust
        self.max_u2 = self.max_thrust * self.moment_arm
        self.max_u3 = self.max_thrust * self.moment_arm
        self.max_u4 = 2 * self.torque_coeff * self.max_omega**2 

        self.action_space = gym.spaces.Box(0, self.max_rpm, shape=(4,))
        self.observation_space = None

        print("Simulation parameters:")
        print("Aircraft mass: ", self.ac_mass)
        print("Gravity: ", self.sim_gravity)
        print("Torque coefficient: ", self.torque_coeff)
        print("Thrust coefficient: ", self.thrust_coeff)
        print("Maximum RPM: ", self.max_rpm)
        print("Hover RPM: ", self.hov_rpm)
        print("Hover Omega: ", self.hov_omega)
        print("Action bandwidth: ", self.action_bandwidth)
        print("Maximum U1: ", self.max_u1)
        print("Maximum U2: ", self.max_u2)
        print("Maximum U3: ", self.max_u3)
        print("Maximum U4: ", self.max_u4)
        print()
    
    def get_xyz(self):
        xyz = self.iris.state[:3,:]
        return xyz.flatten().tolist()
    
    def get_zeta(self):
        q = self.iris.state[3:7,:]
        zeta = self.iris.q_to_euler(q)
        return zeta.flatten().tolist()

    def get_uvw(self):
        uvw = self.iris.state[7:10,:]
        return uvw.flatten().tolist()
    
    def get_pqr(self):
        pqr = self.iris.state[10:,:]
        return pqr.flatten().tolist()
    
    def get_xyz_dot(self):
        Q_inv = self.iris.q_conj(self.iris.state[3:7])
        xyz_dot = self.iris.q_mult(Q_inv).dot(self.iris.q_mult(np.vstack([self.iris.zero, self.iris.state[7:10]])).dot(self.iris.state[3:7]))[1:]
        return xyz_dot.flatten().tolist()

    def get_quaternion(self):
        Q = self.iris.state[3:7].flatten().tolist()
        n = sqrt(sum([q**2 for q in Q]))
        quat = [q / n for q in Q]
        return quat
    
    def q_conj(self, q):
        """
        Returns the conjugate q* of quaternion q
        """
        w, i, j, k = q
        n = sum([k**2 for k in q])
        return [w / n, -i / n, -j / n, -k / n]

    def hamilton_product(self, p, q):
        """
        Calculates the Hamilton product for quaternions p and q
        """
        w1, x1, y1, z1 = p
        w2, x2, y2, z2 = q
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        i = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        j = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        k = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
        return [w, i, j, k]
    
    def get_rotation_matrix(self, q):
        """
        Returns a 3x3 rotation matrix using the unit quaternion q
        """
        r, i, j, k = q
        r1 = np.array([1-2*(j**2+k**2), 2*(i*j-k*r), 2*(i*k+j*r)])
        r2 = np.array([2*(i*j+k*r), 1-2*(i**2+k**2), 2*(j*k-i*r)])
        r3 = np.array([2*(i*k-j*r), 2*(j*k+i*r), 1-2*(i**2+j**2)])
        return np.vstack([r1, r2, r3])

    def inertial_to_body(self, vector):
        """
        Rotates a vector or 3D coordinate expressed in the global frame
        into the aircraft's local frame of reference.
        """
        p = [0] + vector
        q = self.get_quaternion()
        r = self.q_conj(q)
        pq = self.hamilton_product(p, q)
        rpq = self.hamilton_product(r, pq)
        return rpq[1:]
        
    def body_to_inertial(self, vector):
        """
        Rotates a vector or 3D coordinate expressed in the aircraft's 
        local frame of reference into the global frame.
        """
        p = [0]+vector
        q = self.get_quaternion()
        r = self.q_conj(q)
        qp = self.hamilton_product(q, p)
        qpr = self.hamilton_product(qp, r)
        return qpr[1:]
        
    def get_data(self):
        xyz = self.get_xyz()
        zeta = self.get_zeta()
        uvw = self.get_uvw()
        pqr = self.get_pqr()
        return xyz, zeta, uvw, pqr

    def get_rpm(self):
        rpm = self.iris.rpm
        return rpm.tolist()

    def omega_to_rpm(self, omega):
        return omega*30./pi
    
    def rpm_to_omega(self, rpm):
        return rpm*pi/30.
    
    def translate_action(self, action):
        rpm_vals = [self.omega_to_rpm(a * self.action_bandwidth + self.hov_omega) for a in action]
        return rpm_vals
    
    def set_action_frequency(self, ctrl_dt):
        self.ctrl_dt = ctrl_dt
        self.sim_steps = int(self.ctrl_dt/self.sim_dt)
        self._max_episode_steps = int(self.T/self.ctrl_dt)

    def reward(self, state, action):
        """
        Override this function
        """
        pass
    
    def terminal(self, state):
        if self.curr_dist >= self.max_dist: return True
        elif self.t*self.ctrl_dt > self.T: return True
        else: return False

    def get_state_obs(self, state, action, normalized_rpm):
        xyz, sin_zeta, cos_zeta, xyz_dot, pqr = state
        state_obs = xyz + sin_zeta + cos_zeta + xyz_dot + pqr
        xyz_obs = self.inertial_to_body([x - g for x, g in zip(xyz, self.goal_xyz)])
        zeta_obs = [sz - sin(g) for sz, g in zip(sin_zeta, self.goal_zeta)]+[cz - cos(g) for cz, g in zip(cos_zeta, self.goal_zeta)]
        vel_obs = self.inertial_to_body([u - g for u, g in zip(xyz_dot, self.goal_xyz_dot)])+[p - g for p, g in zip(pqr, self.goal_pqr)]
        curr_tar_obs = xyz_obs+zeta_obs+vel_obs
        next_state = state_obs + curr_tar_obs + normalized_rpm + [self.t*self.ctrl_dt]
        return next_state
    
    def set_current_dists(self, state, action):
        xyz, sin_zeta, cos_zeta, xyz_dot, pqr = state
        self.curr_dist = np.linalg.norm([x - g for x, g in zip(xyz, self.goal_xyz)])
        self.curr_att_sin = np.linalg.norm([sz - sin(g) for sz, g in zip(sin_zeta, self.goal_zeta)])
        self.curr_att_cos = np.linalg.norm([cz - cos(g) for cz, g in zip(cos_zeta, self.goal_zeta)])
        self.curr_vel = np.linalg.norm([x - g for x, g in zip(xyz_dot, self.goal_xyz_dot)])
        self.curr_ang = np.linalg.norm([x - g for x, g in zip(pqr, self.goal_pqr)])
    
    def step(self, action):
        #print("raw action: ", action)
        commanded_omega = np.array(self.hov_omega) + self.action_bandwidth * action
        print("old action: ", self.omega_to_rpm(commanded_omega))
        #print("action: ", commanded_rpm)
        #print("curr state: ", self.get_data())
        #print("stepping for {} time steps".format(self.sim_steps))
        for _ in range(self.sim_steps):
            xyz, zeta, uvw, pqr = self.iris.step(commanded_omega)
        xyz = xyz.flatten().tolist()
        zeta = zeta.flatten().tolist()
        uvw = uvw.flatten().tolist()
        pqr = pqr.flatten().tolist()
        xyz_dot = self.get_xyz_dot()
        print("old state: ", xyz, zeta, uvw, pqr)
        #print("new state: ", xyz, zeta, uvw, pqr)
        #input()
        sin_zeta = [sin(z) for z in zeta]
        cos_zeta = [cos(z) for z in zeta]
        curr_omega = self.get_rpm()
        normalized_omega = [omega/self.max_omega for omega in curr_omega]
        #print("new")
        #print(self.curr_dist)
        #print(self.curr_att_sin)
        #print(self.curr_att_cos)
        #print(self.curr_vel)
        #print(self.curr_ang)
        self.set_current_dists((xyz, sin_zeta, cos_zeta, xyz_dot, pqr), commanded_omega)
        reward, info = self.reward((xyz, sin_zeta, cos_zeta, xyz_dot, pqr), commanded_omega)
        self.t += 1
        done = self.terminal((xyz, zeta, xyz_dot, pqr))
        obs = self.get_state_obs((xyz, sin_zeta, cos_zeta, xyz_dot, pqr), commanded_omega, normalized_omega)
        #print("obs step: ", obs)
        self.set_prev_dists((xyz, sin_zeta, cos_zeta, xyz_dot, pqr), commanded_omega)
        return obs, reward, done, info

    def set_prev_dists(self, state, action):
        xyz, sin_zeta, cos_zeta, xyz_dot, pqr = state
        self.prev_dist = self.curr_dist
        self.prev_att_sin = self.curr_att_sin
        self.prev_att_cos = self.curr_att_cos
        self.prev_vel = self.curr_vel
        self.prev_ang = self.curr_ang
        self.prev_xyz = xyz
        self.prev_zeta = [acos(z) for z in cos_zeta]
        self.prev_xyz_dot = xyz_dot
        self.prev_pqr = pqr
        self.prev_action = action

    def reset(self):
        self.t = 0
        self.iris.reset()
        xyz, zeta, _, pqr = self.get_data()
        #print(xyz)
        #print(zeta)
        #print(pqr)
        xyz_dot = self.get_xyz_dot()
        #print(xyz_dot)
        sin_zeta = [sin(z) for z in zeta]
        cos_zeta = [cos(z) for z in zeta]
        curr_omega = self.get_rpm()
        normalized_omega = [omega/self.max_omega for omega in curr_omega]
        next_state = [xyz, sin_zeta, cos_zeta, xyz_dot, pqr, normalized_omega]
        print("old reset")
        print(next_state)
        self.prev_action = self.hov_rpm_
        self.prev_xyz_dot = [0., 0., 0.]
        self.prev_pqr = [0., 0., 0.]
        return next_state

    def reset_to_custom_state(self, xyz, zeta, uvw, pqr, rpm):
        self.iris.reset()
        self.iris.set_state(xyz, zeta, uvw, pqr)
        self.iris.set_rpm(rpm)
        xyz_dot = self.get_xyz_dot()
        self.t = 0
        sin_zeta = [sin(z) for z in zeta]
        cos_zeta = [cos(z) for z in zeta]
        curr_rpm = self.get_rpm()
        normalized_rpm = [rpm/self.max_rpm for rpm in curr_rpm]
        next_state = [xyz, sin_zeta, cos_zeta, xyz_dot, pqr, normalized_rpm]
        self.prev_action = curr_rpm
        self.prev_xyz_dot = xyz_dot
        self.prev_pqr = pqr
        return next_state

    def render(self, mode='human', close=False):
        if not self.init_rendering:
            self.ani = ani_gl.VisualizationGL(name="Static Waypoint")
            self.init_rendering = True
        self.ani.draw_quadrotor(self)
        self.ani.draw_label("Time: {0:.2f}".format(self.t*self.ctrl_dt),
            (self.ani.window.width // 2, 20.0))