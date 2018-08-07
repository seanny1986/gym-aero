from gym.envs.registration import register

register(
    id='hover-v0',
    entry_point='gym_foo.envs:HoverEnv',
)
register(
    id='static-waypoint-v0',
    entry_point='gym_foo.envs:StaticWaypoint',
)
register(
    id='random-waypoint-v0',
    entry_point='gym_foo.envs:RandomWaypoint',
)
register(
    id='model-training-v0',
    entry_point='gym_foo.envs:ModelTraining',
)
