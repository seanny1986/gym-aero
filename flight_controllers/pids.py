import numpy as np
from math import cos

class PID:
    def __init__(self, gains, control_matrix, u_max, w_max, mg, thresh, dt=1e-3):
        """
        K_p, K_i, K_d are 4x4 matrices of the form:

        [[Kz,  0,   0,   0],
        [0,    Kp,  0,   0],
        [0,    0,   Kq,  0],
        [0,    0,   0,   Kr]]

        Where Kz, Kp, Kq, and Kr are gains for the altitude, and p, q and r angular rates, respectively.
        
        control_matrix is a 4x4 matrix as outlined above.
        """

        self.gains = gains
        self.control_matrix = control_matrix
        self.dt = dt
        self.inv_control_matrix = np.linalg.inv(control_matrix)
        self.integral = np.zeros((4,1))
        self.previous_error = np.zeros((4,1))
        self.u_max = u_max
        self.w_max_sq = w_max ** 2
        self.mg = mg
        self.thresh = thresh

    def reset(self):
        self.integral = np.zeros((4,1))         # reset integral value
        self.previous_error = np.zeros((4,1))   # reset derivative value
    
    def pid(self, target, state, gains):
        error = target - state                  # calc e(t)
        de = error - self.previous_error        # change in error
        mask = np.abs(error) < self.thresh      # mask to determine if our error is below a certain thresh
        self.integral += error*mask             # only integrate if we're within a certain radius of the target
        p = gains["K_p"].dot(error)                 # p term
        i = self.dt*gains["K_i"].dot(self.integral) # i term
        d = (1/self.dt)*gains["K_d"].dot(de)        # d term
        u = p + i + d                           # u(t) = f(e(t))
        self.previous_error = error             # set previous error for derivative calc. Will remove this in later versions
        return u


class RatesPID(PID):
    """
    Implements a rate PID controller for a quadrotor. We want to solve the equation:
    
    [[u1],      [[kt,   kt,     kt,     kt],    [[w_1^2],
    [u2],   =   [0.,    lkt,    0.,   -lkt],     [w_2^2],
    [u3],       [-lkt,  0.,     lkt,    0.],     [w_3^2],
    [u4]]       [-kq,   kq,    -kq,     kq]]     [w_4^2]]
    
    Where u1, u2, u3, and u4, correspond to the commanded thrust, roll, pitch, and yaw rates respectively.
    This controller takes in a target Z value, and target angular rates P, Q, R, and outputs an action
    that controls the aircraft to reach that target.
    """
    
    def __init__(self, gains, control_matrix, u_max, w_max, mg, thresh, dt=1e-3):
        super(RatesPID, self).__init__(gains, control_matrix, u_max, w_max, mg, thresh, dt)
        
    def w(self, target, state):
        # this code pulls out the state inputs we need for this PID controller
        ######################################################################
        _, zeta, xyz_dot, pqr = state
        phi, theta, _ = zeta
        X = np.array([[xyz_dot[-1]],
                        [pqr[0]],
                        [pqr[1]],
                        [pqr[2]]])                                      # probably a better / faster way to do this. Maybe re-write in C?
        ######################################################################
        U = self.pid(target, X, self.gains["vel"])                      # calculate U(t). Switch for NED
        U[0] += self.mg                                                 # U1 = (-f(e(t)) + mg) / (cos(phi) cos(theta)). Compensate for weight.
        U[0] /= cos(phi) * cos(theta)                                   # See above. Need to compensate for rol and pitch in thrust output.
        V = np.vstack([np.clip(u, n[0], n[1]) for u, n in zip(U, self.u_max)])  # clip values to interval
        W = np.sqrt(np.clip(self.inv_control_matrix.dot(V), 0, self.w_max_sq))  # find omega values
        return W


class AttitudePID(PID):
    """
    Implements a position PID controller for a quadrotor. We want to solve the equation:
    
    [[u1],      [[kt,   kt,     kt,     kt],    [[w_1^2],
    [u2],   =   [0.,    lkt,    0.,   -lkt],     [w_2^2],
    [u3],       [-lkt,  0.,     lkt,    0.],     [w_3^2],
    [u4]]       [-kq,   kq,    -kq,     kq]]     [w_4^2]]
    
    Where u1, u2, u3, and u4, correspond to the commanded thrust, roll, pitch, and yaw rates respectively.
    This controller takes in a target Z value, and target angular rates P, Q, R, and outputs an action
    that controls the aircraft to reach that target.
    """

    def __init__(self, gains, control_matrix, u_max, w_max, mg, thresh, dt=1e-3):
        super(AttitudePID, self).__init__(gains, control_matrix, u_max, w_max, mg, thresh, dt)
    
    def w(self, target, state):
        # this code pulls out the state inputs we need for this PID controller
        ######################################################################
        xyz, zeta, xyz_dot, pqr = state
        phi, theta, _ = zeta
        X = np.array([[xyz[-1]],
                        [zeta[0]],
                        [zeta[1]],
                        [zeta[2]]])                                      # probably a better / faster way to do this. Maybe re-write in C?
        U = self.pid(target, X, self.gains["att"])
        U[0] /= cos(phi) * cos(theta)

        ######################################################################
        X = np.array([[xyz_dot[-1]],
                        [pqr[0]],
                        [pqr[1]],
                        [pqr[2]]]) 
        U = self.pid(X, U.reshape(-1, 1), self.gains["vel"])                      # calculate U(t)
        U[0] += self.mg                                                 # U1 = (-f(e(t)) + mg) / (cos(phi) cos(theta)). Compensate for weight.
        
        V = np.vstack([np.clip(u, n[0], n[1]) for u, n in zip(U, self.u_max)])  # clip values to interval
        W = np.sqrt(np.clip(self.inv_control_matrix.dot(V), 0, self.w_max_sq))  # find omega values
        return W


class PositionPID(PID):
    """
    Implements a position PID controller for a quadrotor. We want to solve the equation:
    
    [[u1],      [[kt,   kt,     kt,     kt],    [[w_1^2],
    [u2],   =   [0.,    lkt,    0.,   -lkt],     [w_2^2],
    [u3],       [-lkt,  0.,     lkt,    0.],     [w_3^2],
    [u4]]       [-kq,   kq,    -kq,     kq]]     [w_4^2]]
    
    Where u1, u2, u3, and u4, correspond to the commanded thrust, roll, pitch, and yaw rates respectively.
    This controller takes in a target Z value, and target angular rates P, Q, R, and outputs an action
    that controls the aircraft to reach that target.
    """

    def __init__(self, gains, control_matrix, u_max, w_max, mg, thresh, dt=1e-3):
        super(PositionPID, self).__init__(gains, control_matrix, u_max, w_max, mg, thresh, dt)
        
    def w(self, target, state):
        xyz, zeta, xyz_dot, pqr = state
        phi, theta, _ = zeta
        X = np.array([[xyz[0]],
                        [xyz[1]]
                        [zeta[2]]])                          
        U = self.pid(target[1:], X, self.gains["pos"])        
        
        X = np.array([[xyz[2]],
                        [zeta[0]],
                        [zeta[1]],
                        [zeta[2]]])
        U = np.insert(U, target[0], 0, axis=0) 
        U = self.pid(U.reshape(-1,1), X, self.gains["att"])
        U[0] /= cos(phi) * cos(theta)   
        
        X = np.array([[xyz_dot[2]],
                        [pqr[0]],
                        [pqr[1]],
                        [pqr[2]]]) 
        U = self.pid(X, U.reshape(-1, 1), self.gains["vel"])
        U[0] += self.mg                                                 # U1 = (-f(e(t)) + mg) / (cos(phi) cos(theta)). Compensate for weight.
        
        V = np.vstack([np.clip(u, n[0], n[1]) for u, n in zip(U, self.u_max)])  # clip values to interval
        W = np.sqrt(np.clip(self.inv_control_matrix.dot(V), 0, self.w_max_sq))  # find omega values
        return W