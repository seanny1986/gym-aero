import numpy as np
from math import sin, cos, tan, sqrt

class Quadrotor:
    """
        6DOF rigid body, non-linear EOM solver for a '+' configuration quadrotor. Aircraft is modeled
        with an East-North-Up axis system for convenience when plotting. This means thrust is positive
        in the body-axis z-direction, and the gravity vector is negative in the inertial axis 
        z-direction. The aircraft comes with a config file that includes the necessary parameters. For
        a description of what each parameter means, please check this file.

        I've chosen a representation for the rotation matrices that makes it easy to see what I'm doing;
        it shouldn't have a huge impact on performance since we don't have much in the way of graphics.
        For an 'x' config quadrotor, calculate thrust and moments due to the motors as normal, and then 
        rotate these vectors by pi/4 around the body z-axis. Thrust should be unaffected, but the moments
        will be, since the moment arm to the COM will change.

        You can also use this sim for standard fixed-wing aircraft or rockets by implementing new force
        and torque methods. For example, for a fixed wing, you could implement a strip theory aerodynamics 
        solver to get lift and drag in the body frame, VLM, or a linearized method. For a rocket you would 
        use standard rocket thrust equations, and update the mass of the vehicle (which has its own ODE 
        and is included in the state space). Quaternion rotations are probably better for rockets though,
        since the rotation matrices have a singularity at pitch +-90 degrees.

        -- Sean Morrison, 2018
    """
    
    def __init__(self, params):
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
        self.g = params["g"]
        self.dt = params["dt"]

        self.J = np.array([[self.Jxx,   0.,         0.],
                            [0.,    self.Jyy,       0.],
                            [0.,        0.,     self.Jzz]])
        self.xyz = np.array([[0.],
                            [0.],
                            [0.]])
        self.zeta = np.array([[0.],
                            [0.],
                            [0.]])
        self.uvw = np.array([[0.],
                            [0.],
                            [0.]])
        self.pqr = np.array([[0.],
                            [0.],
                            [0.]])
        self.G = np.array([[0.],
                            [0.],
                            [-self.g]])
        self.rpm = np.array([0.0, 0., 0., 0.])

        self.u_to_rpm = np.linalg.inv(np.array([[self.kt, self.kt, self.kt, self.kt],
                                                [0., self.l*self.kt, 0., -self.l*self.kt],
                                                [-self.l*self.kt, 0., self.l*self.kt, 0.],
                                                [-self.kq, self.kq, -self.kq, self.kq]]))

        # important physical limits
        self.hov_rpm = sqrt((self.mass*self.g)/self.n_motors/self.kt)
        self.max_rpm = sqrt(1./self.hov_p)*self.hov_rpm
        self.max_thrust = self.kt*self.max_rpm**2
        self.terminal_velocity = sqrt((self.max_thrust+self.mass*self.g)/self.kd)
        self.terminal_rotation = sqrt(self.l*self.max_thrust/self.km)

    def set_state(self, xyz, zeta, uvw, pqr):
        """
            Sets the state space of our vehicle
        """

        self.xyz = xyz.copy()
        self.zeta = zeta.copy()
        self. uvw = uvw.copy()
        self.pqr = pqr.copy()
    
    def get_state(self):
        """
            Returns the current state space
        """
        return self.xyz, self.zeta, self.uvw, self.pqr
    
    def reset(self):
        """
            Resets the initial state of the quadrotor
        """

        self.xyz = np.array([[0.],
                            [0.],
                            [0.]])
        self.zeta = np.array([[0.],
                            [0.],
                            [0.]])
        self.uvw = np.array([[0.],
                            [0.],
                            [0.]])
        self.pqr = np.array([[0.],
                            [0.],
                            [0.]]) 
        self.rpm = np.array([0., 0., 0., 0.])
        return self.get_state()

    def R1(self, zeta):
        """
            Rotation matrix converting body frame linear values to the inertial frame.
            This matrix is orthonormal, so to go from the inertial frame to the body
            frame, we can take the transpose of this matrix. That is, R1^-1 = R1^T.
            These rotations are for an East-North-Up axis system, since matplotlib 
            uses this for plotting. If you wanted to use N-E-D as is more typical in
            aerospace, you would need two additional rotation matrices for plotting -- 
            a pi/2 rotation about the inertial z-axis, and then another pi/2 rotation 
            about the inertial x-axis.
        """
        
        phi = zeta[0,0]
        theta = zeta[1,0]
        psi = zeta[2,0]
        
        R_z = np.array([[cos(psi),      -sin(psi),          0.],
                        [sin(psi),      cos(psi),           0.],
                        [0.,                0.,             1.]])
        R_y = np.array([[cos(theta),        0.,     sin(theta)],
                        [0.,                1.,             0.],
                        [-sin(theta),       0.,     cos(theta)]])
        R_x =  np.array([[1.,               0.,             0.],
                        [0.,            cos(phi),       -sin(phi)],
                        [0.,            sin(phi),       cos(phi)]])
        return R_z.dot(R_y.dot(R_x))

    def R2(self, zeta):
        """
            Euler rates rotation matrix converting body frame angular velocities 
            to the inertial frame. This uses the East-North-Up axis convention, so 
            it looks a bit different to the rates matrix in most aircraft dynamics
            textbooks (which use an N-E-D system).
        """

        theta = zeta[1,0]
        psi = zeta[2,0]
        return np.array([[cos(psi)/cos(theta), sin(psi)/cos(theta), 0.],
                        [-sin(psi),             cos(psi),           0.],
                        [cos(psi)*tan(theta), sin(psi)*tan(theta),  1.]])

    def aero_forces(self):
        """
            Calculates drag in the body xyz axis (E-N-U) due to linear velocity
        """

        mag = np.linalg.norm(self.uvw)
        if mag == 0:
            return np.array([[0.],
                            [0.],
                            [0.]])
        else:
            norm = self.uvw/mag
            return -(self.kd*mag**2)*norm

    def aero_moments(self):
        """
            Models aero moments about the body xyz axis (E-N-U) as a function of angular velocity
        """

        mag = np.linalg.norm(self.pqr)
        if mag == 0:
            return np.array([[0.],
                            [0.],
                            [0.]])
        else:
            norm = self.pqr/mag
            return -(self.km*mag**2)*norm

    def thrust_forces(self, rpm):
        """
            Calculates thrust forces in the body xyz axis (E-N-U)
        """
        
        thrust = self.kt*rpm**2
        f_body_x, f_body_y = 0., 0.
        f_body_z = np.sum(thrust)
        return np.array([[f_body_x],
                        [f_body_y],
                        [f_body_z]])
    
    def thrust_moments(self, rpm):
        """
            Calculates moments about the body xyz axis due to motor thrust and torque
        """

        thrust = self.kt*rpm**2
        t_body_x = self.l*(thrust[1]-thrust[3])
        t_body_y = self.l*(thrust[2]-thrust[0])
        motor_torques = self.kq*rpm**2
        t_body_z = -motor_torques[0]+motor_torques[1]-motor_torques[2]+motor_torques[3]
        return np.array([[t_body_x],
                        [t_body_y],
                        [t_body_z]])
    
    def step(self, control_signal, rpm_commands=True, return_acceleration=False):
        """
            Semi-implicit Euler update of the non-linear equations of motion. Uses the
            matrix form since it's much nicer to work with. Our state space equations 
            are:
            
            v_dot = F_b/m + R1^{-1}G_i - omega x v
            omega_dot = J^{-1}[Q_b - omega x H]
            x_dot = R1*v
            zeta_dot = R2*omega

            Where F_b are the external body forces (thrust+drag) in the body frame, m 
            is the mass of the vehicle, R1^{-1} is the inverse of matrix R1 (since R1
            rotates the body frame to the inertial frame, the inverse rotates the inertial
            to the body frame), G_i is the gravity vector in the inertial frame (0,0,-9.81),
            omega is the angular velocity, v is the velocity, J is the inertia matrix,
            Q_b are the external moments about the body axes system (motor thrust, motor
            torque, and aerodynamic moments), and H is the angular momentum J*omega. I 
            assume J is a diagonal matrix here; that is, the aircraft is symmetrical about 
            the body x and y axes.
            
            Since R1 is an orthornormal matrix:

            R1^{-1} = R1^{T}

            This is not true for the Euler rates matrix R2, so we need to invert it the old
            fashioned way.

            I step the EOMs forward under the assumption that:

            x_{t+1} ~ x_{t}+h*x_dot_{t}

            Where h is the time step. This update is semi-implicit since it updates linear 
            and angular velocities using a forward Euler step, and then updates position and
            attitude using v_{t+1} and omega_{t+1} (as opposed to using v_{t} and omega_{t}).
        """

        if not rpm_commands:
            rpm_sq = self.u_to_rpm.dot(control_signal)
            rpm_sq = np.clip(rpm_sq, 0, self.max_rpm**2)
            rpm = (rpm_sq**0.5).flatten()
        else:
            rpm = control_signal
            rpm = np.clip(rpm, 0., self.max_rpm)
            
        r1 = self.R1(self.zeta)
        r2 = self.R2(self.zeta)
        fm = self.thrust_forces(rpm)
        tm = self.thrust_moments(rpm)
        fa = self.aero_forces()
        ta = self.aero_moments()
        H = self.J.dot(self.pqr)
        uvw_dot = (fm+fa)/self.mass+r1.T.dot(self.G)-np.cross(self.pqr, self.uvw, axis=0)
        pqr_dot = np.linalg.inv(self.J).dot(tm+ta-np.cross(self.pqr, H, axis=0))
        self.uvw += uvw_dot*self.dt
        self.pqr += pqr_dot*self.dt
        xyz_dot = r1.dot(self.uvw)
        zeta_dot = r2.dot(self.pqr)
        self.xyz += xyz_dot*self.dt
        self.zeta += zeta_dot*self.dt
        if not return_acceleration:
            return self.xyz, self.zeta, self.uvw, self.pqr
        else:    
            return self.xyz, self.zeta, self.uvw, self.pqr, xyz_dot, zeta_dot, uvw_dot, pqr_dot