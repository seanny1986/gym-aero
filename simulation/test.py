import numpy as np
import ctypes

import random
from math import pi, sin, cos, tanh, exp, sqrt, acos

import animation_gl as ani_gl
import time

class Quadrotor:
    def __init__(self):
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

        self.T = 1
        self.t = 0
        self.dt = 0.01 #self.iris.get_time_step()
        self.ctrl_dt = 0.05
        self.max_rpm = self.omega_to_rpm(sqrt(self.ac_mass*self.sim_gravity/(2.*self.thrust_coeff)))
        self.hov_rpm = self.omega_to_rpm(sqrt(self.ac_mass*self.sim_gravity/(4.*self.thrust_coeff)))
        self.hov_rpm_ = [self.hov_rpm, self.hov_rpm, self.hov_rpm, self.hov_rpm]
        self.hov_omega = self.hov_rpm*pi/30.
        self.action_bandwidth = 35.

        print("Simulation parameters:")
        print("Aircraft mass: ", self.ac_mass)
        print("Gravity: ", self.sim_gravity)
        print("Torque coefficient: ", self.torque_coeff)
        print("Thrust coefficient: ", self.thrust_coeff)
        print("Maximum RPM: ", self.max_rpm)
        print("Hover RPM: ", self.hov_rpm)
        print("Hover Omega: ", self.hov_omega)
        
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

k = 30./pi

ani = ani_gl.VisualizationGL(name="Test")
quad = Quadrotor()
n = int(quad.ctrl_dt/quad.dt)
print("Number of timesteps per action: ", n)
sim_steps = int(quad.T/quad.ctrl_dt)
print("Testing gravity")
quad.iris.set_init_rpm(quad.hov_rpm, quad.hov_rpm, quad.hov_rpm, quad.hov_rpm)
quad.iris.sim_init()
quad.iris.set_max_rpm(quad.max_rpm)
print("Init RPM: ", quad.get_rpm())
print(quad.hov_rpm)
for i in range(sim_steps):
    quad.iris.sim_step(0., 0., 0., 0., n)
    ani.draw_quadrotor(quad)
    ani.draw_label("Time: {0:.2f}".format(quad.t),
    (ani.window.width // 2, 20.0))
    ani.draw()
    quad.t += quad.ctrl_dt
    time.sleep(0.05)
print("Time: ", quad.t)
print("Final state: ", quad.get_data())
quad.iris.sim_term()
ani.close_window()
print()

print("Testing full throttle")
quad.iris.set_init_rpm(quad.max_rpm, quad.max_rpm, quad.max_rpm, quad.max_rpm)
quad.iris.sim_init()
quad.iris.set_max_rpm(quad.max_rpm)
print("Init RPM: ", quad.get_rpm())
print(quad.hov_rpm)
ani = ani_gl.VisualizationGL(name="Test")
for i in range(sim_steps):
    quad.iris.sim_step(quad.max_rpm, quad.max_rpm, quad.max_rpm, quad.max_rpm, n)
    ani.draw_quadrotor(quad)
    ani.draw_label("Time: {0:.2f}".format(quad.t),
    (ani.window.width // 2, 20.0))
    ani.draw()
    quad.t += quad.ctrl_dt
    time.sleep(0.05)
print("Time: ", quad.t)
print("Final state: ", quad.get_data())
quad.iris.sim_term()
ani.close_window()

print("Testing configuration")
quad.iris.set_init_rpm(quad.hov_rpm, quad.hov_rpm, quad.hov_rpm, quad.hov_rpm)
quad.iris.sim_init()
quad.iris.set_max_rpm(quad.max_rpm)
print("Init RPM: ", quad.get_rpm())
print(quad.hov_rpm)
ani = ani_gl.VisualizationGL(name="Test")
for i in range(sim_steps):
    quad.iris.sim_step(quad.hov_rpm+5*k, quad.hov_rpm, quad.hov_rpm, quad.hov_rpm, n)
    ani.draw_quadrotor(quad)
    ani.draw_label("Time: {0:.2f}".format(quad.t),
    (ani.window.width // 2, 20.0))
    ani.draw()
    quad.t += quad.ctrl_dt
    time.sleep(0.05)
print("Time: ", quad.t)
print("Final state: ", quad.get_data())
quad.iris.sim_term()
ani.close_window()
print()