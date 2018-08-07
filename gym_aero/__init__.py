from gym.envs.registration import register

register(
    id='Hover-v0',
    entry_point='gym_aero.envs:HoverEnv',
)
register(
    id='StaticWaypoint-v0',
    entry_point='gym_aero.envs:StaticWaypoint',
)
register(
    id='RandomWaypoint-v0',
    entry_point='gym_aero.envs:RandomWaypoint',
)
register(
    id='ModelTraining-v0',
    entry_point='gym_aero.envs:ModelTraining',
)
