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
    id='Trajectory-v0',
    entry_point='gym_aero.envs:TrajectoryEnv',

)
