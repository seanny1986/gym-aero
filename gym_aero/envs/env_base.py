import numpy as np
import ctypes

import random
from math import pi, sin, cos, tanh, exp, sqrt, acos

import gym
from gym import error, spaces, utils
from gym.utils import seeding
from simulation import animation_gl as ani_gl
import time


"""
Defines the base class for environments in gym-aero. 
Aircraft is modelled in a NED axis system.
-- Sean Morrison, 2019
"""

class AeroEnv(gym.Env):
    def __init__(self):
        #super(AeroEnv, self).__init__()
        self.iris = ctypes.CDLL("/home/seanny/gym-aero/simulation/quadrotor_sim.so")
        self.iris.sim_step.argtypes = [ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double]
        self.iris.set_init_pos.argtypes = [ctypes.c_double, ctypes.c_double, ctypes.c_double]
        self.iris.set_init_euler.argtypes = [ctypes.c_double, ctypes.c_double, ctypes.c_double]
        self.iris.set_init_vel.argtypes = [ctypes.c_double, ctypes.c_double, ctypes.c_double]
        self.iris.set_init_omega.argtypes = [ctypes.c_double, ctypes.c_double, ctypes.c_double]
        self.iris.set_init_rpm.argtypes = [ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double]
        self.iris.set_min_rpm.argtypes = [ctypes.c_double]
        self.iris.set_max_rpm.argtypes = [ctypes.c_double]

        self.iris.get_x.restype = ctypes.c_double
        self.iris.get_y.restype = ctypes.c_double
        self.iris.get_z.restype = ctypes.c_double

        self.iris.get_phi.restype = ctypes.c_double
        self.iris.get_theta.restype = ctypes.c_double
        self.iris.get_psi.restype = ctypes.c_double

        self.iris.get_u.restype = ctypes.c_double
        self.iris.get_v.restype = ctypes.c_double
        self.iris.get_w.restype = ctypes.c_double

        self.iris.get_p.restype = ctypes.c_double
        self.iris.get_q.restype = ctypes.c_double
        self.iris.get_r.restype = ctypes.c_double

        #self.iris.get_time_step.restype = ctypes.c_float
        self.iris.get_mass.restype = ctypes.c_float
        self.iris.get_gravity.restype = ctypes.c_float
        self.iris.get_torque_coeff.restype = ctypes.c_float
        self.iris.get_thrust_coeff.restype = ctypes.c_float

        self.iris.get_rpm_0.restype = ctypes.c_float
        self.iris.get_rpm_1.restype = ctypes.c_float
        self.iris.get_rpm_2.restype = ctypes.c_float
        self.iris.get_rpm_3.restype = ctypes.c_float

        self.init_rendering = False

        self.ac_mass = self.iris.get_mass()
        self.sim_gravity = self.iris.get_gravity()
        self.torque_coeff = self.iris.get_torque_coeff()
        self.thrust_coeff = self.iris.get_thrust_coeff()

        self.T = None
        self.t = 0
        self.dt = 0.01 #self.iris.get_time_step()
        self.ctrl_dt = 0.05
        self.max_rpm = self.omega_to_rpm(sqrt(self.ac_mass*self.sim_gravity/2./self.thrust_coeff))
        self.hov_rpm = (1/sqrt(2))*self.max_rpm
        self.hov_rpm_ = [self.hov_rpm, self.hov_rpm, self.hov_rpm, self.hov_rpm]
        self.hov_omega = self.hov_rpm*pi/30.
        self.action_bandwidth = 35.*(30./pi)

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
        
    def get_data(self):
        x = self.iris.get_x()
        y = self.iris.get_y()
        z = self.iris.get_z()

        phi = self.iris.get_phi()
        theta = self.iris.get_theta()
        psi = self.iris.get_psi()

        u = self.iris.get_u()
        v = self.iris.get_v()
        w = self.iris.get_w()

        p = self.iris.get_p()
        q = self.iris.get_q()
        r = self.iris.get_r()
        return [x, y, z], [phi, theta, psi], [u, v, w], [p, q, r]

    def get_rpm(self):
        m_0 = self.iris.get_rpm_0()
        m_1 = self.iris.get_rpm_1()
        m_2 = self.iris.get_rpm_2()
        m_3 = self.iris.get_rpm_3()
        return [m_0, m_1, m_2, m_3]

    def omega_to_rpm(self, omega):
        return omega*30./pi
    
    def rpm_to_omega(self, rpm):
        return rpm*pi/30.
    
    def translate_action(self, action):
        rpm_vals = [a*self.action_bandwidth+self.hov_rpm for a in action]
        #rpm_vals = [0. if rpm < 0 else rpm for rpm in rpm_vals]
        #rpm_vals = [self.max_rpm if rpm > self.max_rpm else rpm for rpm in rpm_vals]
        #print([rpm/self.max_rpm for rpm in rpm_vals])
        return rpm_vals

    def reward(self):
        """
        Override this function
        """
        pass
    
    def step(self, action):
        #print("Action: ", rpm_vals)
        steps = int(self.ctrl_dt/self.dt)
        #print("Stepping for {} timesteps".format(steps))
        self.iris.sim_step(action[0], action[1], action[2], action[3], steps)
        #print("Done stepping.")
        #print(self.get_rpm())
        xyz, zeta, uvw, pqr = self.get_data()
        return xyz, zeta, uvw, pqr
    
    def terminal(self):
        """
        Override this function
        """
        pass

    def reset(self):
        #print("Resetting environment.")
        self.iris.sim_term()
        self.iris.set_init_rpm(self.hov_rpm, self.hov_rpm, self.hov_rpm, self.hov_rpm)
        self.iris.sim_init()
        self.iris.set_max_rpm(self.max_rpm)
        xyz, zeta, uvw, pqr = self.get_data()
        self.t = 0
        sin_zeta = [sin(z) for z in zeta]
        cos_zeta = [cos(z) for z in zeta]
        curr_rpm = self.get_rpm()
        normalized_rpm = [rpm/self.max_rpm for rpm in curr_rpm]
        next_state = [xyz, sin_zeta, cos_zeta, uvw, pqr, normalized_rpm]
        self.prev_action = self.hov_rpm_
        self.prev_uvw = [0., 0., 0.]
        self.prev_pqr = [0., 0., 0.]
        return next_state
    
    def set_init_state(self, xyz, zeta, uvw, pqr, rpm):
        self.iris.set_init_pos(xyz[0], xyz[1], xyz[2])
        self.iris.set_init_euler(zeta[0], zeta[1], zeta[2])
        self.iris.set_init_vel(uvw[0], uvw[1], uvw[2])
        self.iris.set_init_omega(pqr[0], pqr[1], pqr[2])
        self.iris.set_init_rpm(rpm[0], rpm[1], rpm[2], rpm[3])

    def reset_to_custom_state(self, xyz, zeta, pqr, uvw, rpm):
        self.iris.sim_term()
        self.iris.set_max_rpm(self.max_rpm)
        self.iris.set_min_rpm(0.)
        self.set_init_state(xyz, zeta, pqr, uvw, rpm)
        self.iris.sim_init()
        xyz, zeta, uvw, pqr = self.get_data()
        self.t = 0
        sin_zeta = [sin(z) for z in zeta]
        cos_zeta = [cos(z) for z in zeta]
        curr_rpm = self.get_rpm()
        normalized_rpm = [rpm/self.max_rpm for rpm in curr_rpm]
        next_state = [xyz, sin_zeta, cos_zeta, uvw, pqr, normalized_rpm]
        self.prev_action = curr_rpm
        self.prev_uvw = uvw
        self.prev_pqr = pqr
        return next_state

    def render(self, mode='human', close=False):
        if not self.init_rendering:
            self.ani = ani_gl.VisualizationGL(name="Static Waypoint")
            self.init_rendering = True
        self.ani.draw_quadrotor(self)
        self.ani.draw_label("Time: {0:.2f}".format(self.t*self.ctrl_dt),
            (self.ani.window.width // 2, 20.0))