import numpy as np
import ctypes

import random
from math import pi, sin, cos, tanh, exp, sqrt, acos

import gym
from gym import error, spaces, utils
from gym.utils import seeding
from simulation import animation_gl as ani_gl
import time
import os

"""
Defines the base class for environments in gym-aero.
Aircraft is modelled in a NED axis system.
-- Sean Morrison, 2019
"""

class AeroEnv(gym.Env):
    def __init__(self):
        #super(AeroEnv, self).__init__()
        path = os.path.expanduser("~")        
        self.iris = ctypes.CDLL(path+"/gym-aero/simulation/quadrotor_sim.so")
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

        self.iris.get_x_dot.restype = ctypes.c_double
        self.iris.get_y_dot.restype = ctypes.c_double
        self.iris.get_z_dot.restype = ctypes.c_double

        self.iris.get_u_dot.restype = ctypes.c_double
        self.iris.get_v_dot.restype = ctypes.c_double
        self.iris.get_w_dot.restype = ctypes.c_double

        self.iris.get_p_dot.restype = ctypes.c_double
        self.iris.get_q_dot.restype = ctypes.c_double
        self.iris.get_r_dot.restype = ctypes.c_double

        self.iris.get_q0.restype = ctypes.c_double
        self.iris.get_q1.restype = ctypes.c_double
        self.iris.get_q2.restype = ctypes.c_double
        self.iris.get_q3.restype = ctypes.c_double

        #self.iris.get_time_step.restype = ctypes.c_float
        self.iris.get_mass.restype = ctypes.c_float
        self.iris.get_gravity.restype = ctypes.c_float
        self.iris.get_torque_coeff.restype = ctypes.c_float
        self.iris.get_thrust_coeff.restype = ctypes.c_float
        self.iris.get_rad.restype = ctypes.c_float

        self.iris.get_rpm_0.restype = ctypes.c_float
        self.iris.get_rpm_1.restype = ctypes.c_float
        self.iris.get_rpm_2.restype = ctypes.c_float
        self.iris.get_rpm_3.restype = ctypes.c_float

        self.init_rendering = False

        self.ac_mass = self.iris.get_mass()
        self.sim_gravity = self.iris.get_gravity()
        self.torque_coeff = self.iris.get_torque_coeff()
        self.thrust_coeff = self.iris.get_thrust_coeff()
        self.moment_arm = self.iris.get_rad()

        self.T = None
        self.t = 0
        self.dt = 0.01 #self.iris.get_time_step()
        self.ctrl_dt = 0.05
        self.sim_steps = int(self.ctrl_dt / self.dt)
        self.max_omega = sqrt(self.ac_mass * self.sim_gravity / 2 / self.thrust_coeff)
        self.max_rpm = self.omega_to_rpm(self.max_omega)
        self.hov_omega = sqrt(self.ac_mass * self.sim_gravity / 4 / self.thrust_coeff)
        self.hov_rpm = self.omega_to_rpm(self.hov_omega)
        self.hov_rpm_ = [self.hov_rpm, self.hov_rpm, self.hov_rpm, self.hov_rpm]
        
        self.max_thrust =  self.thrust_coeff * self.max_omega**2
        self.max_u1 = 4 * self.max_thrust
        self.max_u2 = self.max_thrust * self.moment_arm
        self.max_u3 = self.max_thrust * self.moment_arm
        self.max_u4 = 2 * self.torque_coeff * self.max_omega**2 

        self.action_space = gym.spaces.Box(0, self.max_rpm, shape=(4,))
        self.observation_space = None

        self.squashed_actions = False
        self.action_bandwidth = 35
        if self.squashed_actions: self.squashed = "squashed"
        else: self.squashed = "not_squashed"

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
        x = self.iris.get_x()
        y = self.iris.get_y()
        z = self.iris.get_z()
        return [x, y, z]
    
    def get_zeta(self):
        phi = self.iris.get_phi()
        theta = self.iris.get_theta()
        psi = self.iris.get_psi()
        return [phi, theta, psi]

    def get_uvw(self):
        u = self.iris.get_u()
        v = self.iris.get_v()
        w = self.iris.get_w()
        return [u, v, w]
    
    def get_pqr(self):
        p = self.iris.get_p()
        q = self.iris.get_q()
        r = self.iris.get_r()
        return [p, q, r]

    def get_uvw_dot(self):
        """
        Note: this does not include tangential acceleration.
        """
        u_dot = self.iris.get_u_dot()
        v_dot = self.iris.get_v_dot()
        w_dot = self.iris.get_w_dot()
        return [u_dot, v_dot, w_dot]
    
    def get_pqr_dot(self):
        """
        Note: this does not include tangential acceleration.
        """
        p_dot = self.iris.get_p_dot()
        q_dot = self.iris.get_q_dot()
        r_dot = self.iris.get_r_dot()
        return [p_dot, q_dot, r_dot]
    
    def get_xyz_dot(self):
        x_dot = self.iris.get_x_dot()
        y_dot = self.iris.get_y_dot()
        z_dot = self.iris.get_z_dot()
        return [x_dot, y_dot, z_dot]

    def get_quaternion(self):
        q0 = self.iris.get_q0()
        q1 = self.iris.get_q1()
        q2 = self.iris.get_q2()
        q3 = self.iris.get_q3()
        Q = [q0, q1, q2, q3]
        n = sqrt(sum([q**2 for q in Q]))
        return [q / n for q in Q]
    
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
        if not self.squashed_actions:
            rpm_vals = [self.omega_to_rpm(a * self.action_bandwidth + self.hov_omega) for a in action]
        else:
            rpm_vals = [self.max_rpm * ((a + 1) / 2) for a in action]
        return rpm_vals
    
    def set_action_frequency(self, ctrl_dt):
        self.ctrl_dt = ctrl_dt
        self.sim_steps = int(self.ctrl_dt/self.dt)
        self._max_episode_steps = int(self.T/self.ctrl_dt)
    
    def set_squashed_actions(self, squashed):
        self.squashed_actions = squashed
        if self.squashed_actions: self.squashed = "squashed"
        else: self.squashed = "not_squashed"

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
        xyz_obs = self.inertial_to_body([x - g for x, g in zip(xyz, self.goal_xyz)])
        zeta_obs = [sz - sin(g) for sz, g in zip(sin_zeta, self.goal_zeta)]+[cz - cos(g) for cz, g in zip(cos_zeta, self.goal_zeta)]
        vel_obs = self.inertial_to_body([u - g for u, g in zip(xyz_dot, self.goal_xyz_dot)])+[p - g for p, g in zip(pqr, self.goal_pqr)]
        curr_tar_obs = xyz_obs + zeta_obs + vel_obs
        next_state = curr_tar_obs + normalized_rpm + [self.t*self.ctrl_dt]
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
        commanded_rpm = self.translate_action(action)
        #print("ctrl dt: ", self.ctrl_dt)
        #print("sim dt: ", self.dt)
        #print("sim steps: ", self.sim_steps)
        #print("new action: ", commanded_rpm)
        #print("action: ", commanded_rpm)
        #print("curr state: ", self.get_data())
        #print("stepping for {} time steps".format(self.sim_steps))
        self.iris.sim_step(commanded_rpm[0], commanded_rpm[1], commanded_rpm[2], commanded_rpm[3], self.sim_steps)
        xyz, zeta, uvw, pqr = self.get_data()
        xyz_dot = self.get_xyz_dot()
        #print("new state: ", xyz, zeta, uvw, pqr)
        #input()
        sin_zeta = [sin(z) for z in zeta]
        cos_zeta = [cos(z) for z in zeta]
        curr_rpm = self.get_rpm()
        normalized_rpm = [rpm/self.max_rpm for rpm in curr_rpm]
        self.set_current_dists((xyz, sin_zeta, cos_zeta, xyz_dot, pqr), commanded_rpm)
        reward, info = self.reward((xyz, sin_zeta, cos_zeta, xyz_dot, pqr), commanded_rpm)
        self.t += 1
        done = self.terminal((xyz, zeta, xyz_dot, pqr))
        obs = self.get_state_obs((xyz, sin_zeta, cos_zeta, xyz_dot, pqr), commanded_rpm, normalized_rpm)
        self.set_prev_dists((xyz, sin_zeta, cos_zeta, xyz_dot, pqr), commanded_rpm)
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
        self.iris.sim_term()
        self.iris.set_init_rpm(self.hov_rpm, self.hov_rpm, self.hov_rpm, self.hov_rpm)
        self.iris.sim_init()
        self.iris.set_max_rpm(self.max_rpm)
        self.iris.sim_step(self.hov_rpm, self.hov_rpm, self.hov_rpm, self.hov_rpm, 1)
        xyz, zeta, _, pqr = self.get_data()
        xyz_dot = self.get_xyz_dot()
        sin_zeta = [sin(z) for z in zeta]
        cos_zeta = [cos(z) for z in zeta]
        curr_rpm = self.get_rpm()
        normalized_rpm = [rpm/self.max_rpm for rpm in curr_rpm]
        next_state = [xyz, sin_zeta, cos_zeta, xyz_dot, pqr, normalized_rpm]
        #print("new reset")
        #print(next_state)
        self.prev_action = self.hov_rpm_
        self.prev_xyz_dot = [0., 0., 0.]
        self.prev_pqr = [0., 0., 0.]
        return next_state
    
    def set_init_state(self, xyz, zeta, uvw, pqr, rpm):
        self.iris.set_init_pos(xyz[0], xyz[1], xyz[2])
        self.iris.set_init_euler(zeta[0], zeta[1], zeta[2])
        self.iris.set_init_vel(uvw[0], uvw[1], uvw[2])
        self.iris.set_init_omega(pqr[0], pqr[1], pqr[2])
        self.iris.set_init_rpm(rpm[0], rpm[1], rpm[2], rpm[3])

    def reset_to_custom_state(self, xyz, zeta, uvw, pqr, rpm):
        self.iris.sim_term()
        self.set_init_state(xyz, zeta, uvw, pqr, rpm)
        self.iris.set_init_rpm(rpm[0], rpm[1], rpm[2], rpm[3])
        self.iris.sim_init()
        #self.iris.set_max_rpm(self.max_rpm)
        self.iris.sim_step(rpm[0], rpm[1], rpm[2], rpm[3], 1)
        xyz, zeta, uvw, pqr = self.get_data()
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
        self.ani.draw_goal([0, 0, 0], color=(1, 0, 0))
        self.ani.draw_quadrotor(self)
        self.ani.draw_label("Time: {0:.2f}".format(self.t*self.ctrl_dt),
            (self.ani.window.width // 2, 20.0))
    
    def save_video(self, frame_name, fname):
        self.ani.save_video(frame_name, fname)

    def close(self):
        self.iris.sim_term()