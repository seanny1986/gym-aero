from gym_aero.envs import env_base
from math import sin, cos, acos, pi
from flight_controllers import pids as pids
import gym
import numpy as np

"""
Base environment for using a neural net controller with a tuned quadrotor PID controller. The
control network outputs an error value, which we then use to build a target altitude and angular
rates. From there, the PID controller takes N steps, where N is (NN_frequency/sim_frequency).
"""

class PIDEnv(env_base.AeroEnv):
    def __init__(self):
        super(PIDEnv, self).__init__()

        # Gains matrices for PID controller
        K_p = np.eye(4)
        K_i = np.zeros((4,4))
        K_d = np.eye(4)
        K_p[0,0] = -100
        K_d[0,0] = -0.85

        K_p[1,1] = 2.7/10
        K_i[1,1] = 0
        K_d[1,1] = 0.01/5

        K_p[2,2] = 2.7/10
        K_i[2,2] = 0
        K_d[2,2] = 0.01/5

        K_p[3,3] = 2.7/10
        K_i[3,3] = 0
        K_d[3,3] = 0.01/5

        # control matrix
        l = self.moment_arm
        kt = self.thrust_coeff
        kq = self.torque_coeff
        lkt = l * kt
        self.Q = np.array([[kt, kt, kt, kt],
                            [0, -lkt, 0, lkt],
                            [lkt, 0, -lkt, 0],
                            [-kq, kq, -kq, kq]])

        # max and min values for force U1 and torques U2, U3, U4
        u_max = np.array([[0, self.max_u1],
                  [-self.max_u2, self.max_u2],
                  [-self.max_u3, self.max_u3],
                  [-self.max_u2, self.max_u4]])

        # integration thresholds for PIDs (i.e. only integrate if abs(e(t)) < thresh)
        thresh = np.array([[0.25],
                            [10*(2*pi/360)],
                            [10*(2*pi/360)],
                            [10*(2*pi/360)]])

        mg = self.ac_mass * 9.81
        w_max = self.max_omega
        gains = {"vel" : {"K_p" : K_p,
                            "K_i" : K_i,
                            "K_d" : K_d,}}
        self.controller = pids.RatesPID(gains, self.Q, u_max, w_max, mg, thresh)

    def get_state(self, state):
        xyz, _, _, pqr = state
        X = np.array([[xyz[-1]],
                    [pqr[0]],
                    [pqr[1]],
                    [pqr[2]]])
        return X

    def pid_step(self, errors):
        xyz, zeta, uvw, pqr = self.get_data()
        dz_dot, dp, dq, dr = errors
        xyz_dot = self.get_xyz_dot()
        targ_z_dot = dz_dot + xyz_dot[-1]
        targ_p = dp + pqr[0]
        targ_q = dq + pqr[1]
        targ_r = dr + pqr[2]
        targ = np.array([[targ_z_dot],
                        [targ_p],
                        [targ_q],
                        [targ_r]])
        self.controller.reset()
        for _ in range(self.sim_steps):
            w = self.controller.w(targ, (xyz, zeta, xyz_dot, pqr))
            commanded_rpm = self.omega_to_rpm(w)
            self.iris.sim_step(commanded_rpm[0], commanded_rpm[1], commanded_rpm[2], commanded_rpm[3], 1)
            xyz, zeta, uvw, pqr = self.get_data()
            xyz_dot = self.get_xyz_dot()
        return xyz, zeta, uvw, pqr
    
    def reset(self):
        self.controller.reset()
        obs = super(PIDEnv, self).reset()
        return obs