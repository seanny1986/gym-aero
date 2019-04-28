"""
        Config file for the aircraft. We define the following parameters:
        
        mass = the mass of the vehicle in kg
        prop_radius = the radius of the propellers in meters (this is cosmetic only, no momentum theory)
        n_motors = number of motors on the vehicle
        hov_p = the % max thrust at which we hover. Typically 50%
        l = the length between the centre of mass and the centre of the prop disk (i.e. arm length)
        Jxx = the mass moment of inertia about the x-axis (roll)
        Jyy = the mass moment of inertia about the y-axis (pitch)
        Jzz = the mass moment of inertia about the z-axis (yaw)
        kt = motor thrust coefficient
        kq = motor torque coefficient
        kd = aerodynamic drag coefficient
        km = aerodynamic moment coefficient
        g = gravitational acceleration (positive in config, corrected in simulated)
        dt = solver time step
    """

params = {"mass":0.65,
        "prop_radius": 0.06,
        "n_motors": 4,
	"hov_p": 0.5,
        "l": 0.23,
        "Jxx": 7.5e-3,
        "Jyy": 7.5e-3,
        "Jzz": 1.3e-2,
        "kt": 3.13e-5,
        "kq": 7.5e-7,
        "kd": 9e-3,
        "km": 9e-4,
        "kw": 1/0.18,
        "g": 9.81,
        "dt": 0.05}
