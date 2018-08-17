import numpy as np
from math import sin, cos, acos, sqrt, atan2, asin

class Quadrotor:
    """
        Higher fidelity quadrotor simulation using quaternion rotations and rk4.
        For a description of the aircraft parameters, please see the config file.

        -- Sean Morrison, 2018
    """

    def __init__(self, params):

        # load aircraft parameters. See config file for a description of these.
        self.mass = params["mass"]
        self.prop_radius = params["prop_radius"]
        self.n_motors = params["n_motors"]
        self.hov_p = params["hov_p"]
        self.l = params["l"]
        self.Jxx = params["Jxx"]
        self.Jyy = params["Jyy"]
        self.Jzz = params["Jzz"]
        self.kt = params["kt"]
        self.kq = params["kq"]
        self.kd = params["kd"]
        self.km = params["km"]
        self.kw = params["kw"]
        self.g = params["g"]
        self.dt = params["dt"]

        # inertia tensor. Our aircraft is symmetric along x and y axes, so this is a diagonal matrix.
        self.J = np.array([[self.Jxx, 0., 0.],
                            [0., self.Jyy, 0.],
                            [0., 0., self.Jzz]])

        # gravity quaternion.
        self.G_q = np.array([[0.],
                            [0.],
                            [0.],
                            [-self.g]])

        # all rotation math handled by quaternions. This is secretly part of the state space. State
        # vector is [xyz, q, uvw, pqr]^T, all in a single column vector.
        self.state = np.array([[0.],
                            [0.],
                            [0.],
                            [1.],
                            [0.],
                            [0.],
                            [0.],
                            [0.],
                            [0.],
                            [0.],
                            [0.],
                            [0.],
                            [0.]])

        # we use this matrix to convert from a thrust/moment input to an rpm input.
        self.u_to_rpm = np.linalg.inv(np.array([[self.kt, self.kt, self.kt, self.kt],
                                                [0., self.l*self.kt, 0., -self.l*self.kt],
                                                [-self.l*self.kt, 0., self.l*self.kt, 0.],
                                                [-self.kq, self.kq, -self.kq, self.kq]]))

        # this matrix lets us calculate the 6x1 forces and moments vector using the rpm.
        self.rpm_to_u = np.array([[0., 0., 0., 0.],
                                [0., 0., 0., 0.],
                                [self.kt, self.kt, self.kt, self.kt],
                                [0., self.l*self.kt, 0., -self.l*self.kt],
                                [-self.l*self.kt, 0., self.l*self.kt, 0.],
                                [-self.kq, self.kq, -self.kq, self.kq]])

        # important physical limits
        self.hov_rpm = sqrt((self.mass*self.g)/self.n_motors/self.kt)
        self.max_rpm = sqrt(1./self.hov_p)*self.hov_rpm
        self.max_thrust = self.kt*self.max_rpm

        # action space
        self.rpm = np.array([self.hov_rpm, self.hov_rpm, self.hov_rpm, self.hov_rpm])
        #print(self.rpm)
        # rough velocity and rotation limits.
        self.terminal_velocity = sqrt((self.max_thrust+self.mass*self.g)/self.kd)
        self.terminal_rotation = sqrt(self.l*self.max_thrust/self.km)

        # preallocate memory here
        self.zero = np.array([[0.]])
        self.zero_array = np.array([[0.],[0.],[0.]])
        self.inv_quat = np.array([[1],[-1],[-1],[-1]])

    def set_state(self, xyz, zeta, uvw, pqr):
        """
            Sets the state space of our vehicle
        """

        q = self.euler_to_q(zeta)
        self.state = np.vstack([xyz, q, uvw, pqr])

    def set_rpm(self, rpm):
        """
            Sets current RPM value
        """

        self.rpm = rpm

    def get_state(self):
        """
            Returns the current state space
        """

        xyz = self.state[0:3]
        q = self.state[3:7]
        zeta = self.q_to_euler(q)
        uvw = self.state[7:10]
        pqr = self.state[10:13]
        return xyz, zeta, uvw, pqr

    def reset(self):
        """
            Resets the initial state of the quadrotor
        """

        xyz = np.array([[0.],
                        [0.],
                        [0.]])
        q = np.array([[1.],
                        [0.],
                        [0.],
                        [0.]])
        uvw = np.array([[0.],
                        [0.],
                        [0.]])
        pqr = np.array([[0.],
                        [0.],
                        [0.]])
        zeta = self.q_to_euler(q)
        self.rpm = np.array([self.hov_rpm, self.hov_rpm, self.hov_rpm, self.hov_rpm])
        self.state = np.vstack([xyz, q, uvw, pqr])
        self.t = 0
        return xyz, zeta, uvw, pqr

    def q_norm(self, q):
        """
            Quaternion rotations rely on a unit quaternion. To ensure
            this is the case, we normalize here.
        """

        return q/np.linalg.norm(q)

    def q_mult(self, p):
        """
            One way to compute the Hamilton product is usin Q(p)q, where Q(p) is
            the below 4x4 matrix, and q is a 4x1 quaternion. I decided not to do
            the full multiplication here, and instead return Q(p).
        """

        p0, p1, p2, p3 = p
        return np.array([[p0, -p1, -p2, -p3],
                        [p1, p0, -p3, p2],
                        [p2, p3, p0, -p1],
                        [p3, -p2, p1, p0]]).reshape(4,-1)

    def q_conj(self, q):
        """
            Returns the conjugate q* of quaternion q. q* = q'/|q|, where q is the
            magnitude, and q' is the inverse: q' = [p0, -p1, -p2, -p3]^T. Since we
            always normalize after updating q, we should always have a unit
            quaternion. This means we don't have to normalize in this routine. That
            is, for a unit quaternion, q* = q'
        """

        return self.inv_quat*q

    def q_to_euler(self, q):
        """
            Convert quaternion q to a set of angles zeta. We do all of the heavy
            lifting with quaternions, and then return the Euler angles since they
            are more intuitive.
        """

        q0, q1, q2, q3 = q
        phi = atan2(2.*(q0*q1+q2*q3),q0**2-q1**2-q2**2+q3**2)
        theta = asin(2.*q0*q2-q3*q1)
        psi = atan2(2.*(q0*q3+q1*q2),q0**2+q1**2-q2**2-q3**2)
        return np.array([[phi],
                        [theta],
                        [psi]]).reshape(3,-1)

    def euler_to_q(self, zeta):
        """
            Converts a set of Euler angles to a quaternion. We do this at the very
            start, since we initialize the vehicle with Euler angles zeta.
        """

        phi, theta, psi = zeta
        q0 = cos(phi/2.)*cos(theta/2.)*cos(psi/2.)+sin(phi/2.)*sin(theta/2.)*sin(psi/2.)
        q1 = sin(phi/2.)*cos(theta/2.)*cos(psi/2.)-cos(phi/2.)*sin(theta/2.)*sin(psi/2.)
        q2 = cos(phi/2.)*sin(theta/2.)*cos(psi/2.)+sin(phi/2.)*cos(theta/2.)*sin(psi/2.)
        q3 = cos(phi/2.)*cos(theta/2.)*sin(psi/2.)-sin(phi/2.)*sin(theta/2.)*cos(psi/2.)
        return np.array([[q0],
                        [q1],
                        [q2],
                        [q3]]).reshape(4,-1)

    def aero_forces(self, uvw):
        """
            Calculates drag in the body xyz axis (E-N-U) due to linear velocity
        """

        mag = np.linalg.norm(uvw)               # get vector norm
        if mag == 0:                            # don't want to divide by zero
            return np.array([[0.],              # if velocity is zero, drag is zero
                            [0.],
                            [0.]])
        else:
            unit = uvw/mag                      # normalize velocity vector
            return -(self.kd*mag**2)*unit       # add drag force in negative uvw dir

    def aero_moments(self, pqr):
        """
            Models aero moments about the body xyz axis (E-N-U) as a function of angular velocity
        """

        mag = np.linalg.norm(pqr)               # get vector norm
        if mag == 0:                            # don't want to divide by zero
            return np.array([[0.],              # if velocity is zero, drag is zero
                            [0.],
                            [0.]])
        else:
            unit = pqr/mag                      # normalize velocity vector
            return -(self.km*mag**2)*unit       # add aerodynamic moment in negative pqr dir

    def RK4(self, f):
        """
            RK4 for ODE integration. Argument f is a function f(y), where y can be a
            multidimensional vector [y0, y1, y2, ..., yn]^T. If y is a vector, it should
            be passed as a numpy array.
        """

        return lambda y, dt: (
                lambda dy1: (
                lambda dy2: (
                lambda dy3: (
                lambda dy4: (dy1+2*dy2+2*dy3+dy4)/6.
                )(dt*f(y+dy3))
	            )(dt*f(y+dy2/2.))
	            )(dt*f(y+dy1/2.))
	            )(dt*f(y))

    def solve_accels(self, y):

        # thrust forces and moments, aerodynamic forces and moments
        #print(self.rpm_to_u,self.rpm)
        try:
            fnm = self.rpm_to_u.dot(self.rpm**2)
        except:
            fnm = self.rpm_to_u.dot(self.rpm[0]**2)
        ft = fnm[0:3].reshape((3,1))
        mt = fnm[3:].reshape((3,1))
        fa = self.aero_forces(y[7:10])
        ma = self.aero_moments(y[10:13])

        # sum forces and moments
        forces = ft+fa
        moments = mt+ma

        # calculate angular momentum
        H = self.J.dot(y[10:13])

        # rotate gravity vector from inertial frame to body frame using qpq^-1
        Q = self.q_mult(y[3:7])
        Q_inv = self.q_conj(y[3:7])
        g_b = Q.dot(self.q_mult(self.G_q).dot(Q_inv))[1:]

        # linear and angular accelerations due to thrust and aerodynamic effects
        uvw_dot = forces/self.mass+g_b-np.cross(y[10:13], y[7:10], axis=0)
        pqr_dot = np.linalg.inv(self.J).dot(moments-np.cross(y[10:13], H, axis=0))

        # quaternion time derivative. Lots of vstack calls here which slows things down.
        q_dot = -0.5*self.q_mult(np.vstack([self.zero, y[10:13]])).dot(y[3:7])

        # velocity in the xyz plane (rotated velocity from body to inertial frame)
        xyz_dot = self.q_mult(Q_inv).dot(self.q_mult(np.vstack([self.zero, y[7:10]])).dot(y[3:7]))[1:]

        return np.vstack([xyz_dot, q_dot, uvw_dot, pqr_dot])

    def step(self, control_signal, rpm_commands=True):
        """
            Updating the EOMs using explicit RK4 with quaternion rotations. Should be more
            accurate than quadrotor. In theory, the quaternion rotations should be faster
            to calculate than rotation matrices, and avoid the singularity at pitch +-90
            degrees. In practice, this implementation is slightly slower to calculate because
            we lean heavily on numpy, and copy quite a few arrays using np.vstack. List comp
            might be a faster way of doing this, but afaik would require modifying the RK4
            routine.
        """

        # handle control signal -- we want to clip it to the interval [0, max_rpm]. In some cases
        # we might want to accept a control signal that includes a thrust, and roll, pitch, yaw
        # comands. We handle these by converting them to rpm commands using the conversion matrix
        # that is initialized in the __init__() function.
        if not rpm_commands:
            rpm_sq = self.u_to_rpm.dot(control_signal)
            rpm_sq = np.clip(rpm_sq, 0, self.max_rpm**2)
            rpm_c = (rpm_sq**0.5).flatten()
        else:
            rpm_c = control_signal

        # motor response modeled as first order linear differential equation. Step forward by dt and clip
        w_dot = -self.kw*(self.rpm-rpm_c)
        self.rpm = self.rpm +  w_dot*self.dt
        self.rpm = np.clip(self.rpm, 0., self.max_rpm)

        # step simulation forward
        self.state += self.RK4(self.solve_accels)(self.state, self.dt)

        # normalize quaternion
        self.state[3:7] = self.q_norm(self.state[3:7])

        # return state values
        xyz = self.state[0:3]
        q = self.state[3:7]
        zeta = self.q_to_euler(q)
        uvw = self.state[7:10]
        pqr = self.state[10:]

        return xyz, zeta, uvw, pqr
