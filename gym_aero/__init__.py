from gym.envs.registration import register

register(
    id='Hover-v0',
    entry_point='gym_aero.envs:HoverEnv',
)
register(
    id='StaticWaypoint-v0',
    entry_point='gym_aero.envs:StaticWaypointEnv',
)
register(
    id='RandomWaypoint-v0',
    entry_point='gym_aero.envs:RandomWaypointEnv',
)
register(
    id='ModelTraining-v0',
    entry_point='gym_aero.envs:ModelTrainingEnv',
)
register(
    id='Land-v0',
    entry_point='gym_aero.envs:LandEnv',
)
register(
    id='TargetFollowing-v0',
    entry_point='gym_aero.envs:TargetFollowingEnv',
)
register(
    id='StraightLevelFlight-v0',
    entry_point='gym_aero.envs:StraightLevelFlightEnv',
)
register(
    id='Trajectory-v0',
    entry_point='gym_aero.envs:TrajectoryEnv',
)
register(
    id='Recovery-v0',
    entry_point='gym_aero.envs:RecoveryEnv',
)
register(
    id='Perch-v0',
    entry_point='gym_aero.envs:PerchEnv',
)
