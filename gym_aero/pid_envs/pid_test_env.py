from gym_aero.envs import env_base
from math import sin, cos, acos, pi
import gym
import numpy as np
import random

"""
Env for testing PID rate controller
"""

class PIDTestEnv(env_base.AeroEnv):
    def __init__(self):
        super(PIDTestEnv, self).__init__()
        self.T = None
        self.t = 0
        self.dt = 0.01
        self.ctrl_dt = 0.01
        self.sim_steps = int(self.ctrl_dt/self.dt)

        l = self.moment_arm
        kt = self.thrust_coeff
        kq = self.torque_coeff
        lkt = l * kt
        self.Q = np.array([[kt, kt, kt, kt],
                            [0, -lkt, 0, lkt],
                            [lkt, 0, -lkt, 0],
                            [-kq, kq, -kq, kq]])
    
    def step(self, action):
        commanded_rpm = self.omega_to_rpm(action)
        self.iris.sim_step(commanded_rpm[0], commanded_rpm[1], commanded_rpm[2], commanded_rpm[3], self.sim_steps)
        xyz, zeta, uvw, pqr = self.get_data()
        return xyz, zeta, uvw, pqr
    
    def generate_random_state(self):
        z_dot = random.uniform(-2, 2)
        pqr = [random.uniform(-2, 2) for _ in range(3)]
        return [0., 0., 0.], [0., 0., 0.], [0., 0., z_dot], pqr

    def reset(self):
        self.t = 0
        xyz, zeta, uvw, pqr = self.generate_random_state()
        state = super(PIDTestEnv, self).reset_to_custom_state(xyz, zeta, uvw, pqr, self.hov_rpm_)
        xyz, _, cos_zeta, uvw, pqr, _ = state
        return xyz, [acos(cz) for cz in cos_zeta], uvw, pqr
    
    def render(self, mode='human', close=False):
        super(PIDTestEnv, self).render(mode=mode, close=close)
        self.ani.draw()
        if close:
            self.ani.close_window()
            self.init_rendering = False

