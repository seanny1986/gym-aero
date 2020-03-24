from gym.envs.registration import register

# low level control tasks
register(
    id='Hover-v0',
    entry_point='gym_aero.envs:HoverEnv',
)
register(
    id='RandomWaypointFH-v0',
    entry_point='gym_aero.envs:RandomWaypointFHEnv',
)
register(
    id='RandomWaypointNH-v0',
    entry_point='gym_aero.envs:RandomWaypointNHEnv',
)
register(
    id='Land-v0',
    entry_point='gym_aero.envs:LandEnv',
)
register(
    id='TargetFollow-v0',
    entry_point='gym_aero.envs:TargetFollowEnv',
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
register(
    id='PathFollow-v0',
    entry_point='gym_aero.envs:PathFollowEnv',
)

# hierarchical tasks
register(
    id='BoxWorld-v0',
    entry_point='gym_aero.h_envs:BoxWorld',
)
register(
    id='PlannerBox-v0',
    entry_point='gym_aero.h_envs:PlannerBoxEnv',
)
register(
    id='Planner-v0',
    entry_point='gym_aero.h_envs:PlannerEnv',
)
register(
    id='TrajectoryTerm-v0',
    entry_point='gym_aero.h_envs:TermTrajectoryEnv',
)
register(
    id='RandomWaypointTerm-v0',
    entry_point='gym_aero.h_envs:RandomWaypointEnvTerm',
)
register(
    id='TrajectorySplineTerm-v0',
    entry_point='gym_aero.h_envs:TrajectoryEnvSplineTerm',
)


# multi-task environments
register(
    id='HoverMT-v0',
    entry_point='gym_aero.mt_envs:HoverMTEnv',
)
register(
    id='StaticWaypointMT-v0',
    entry_point='gym_aero.mt_envs:StaticWaypointMTEnv',
)
register(
    id='RandomWaypointMT-v0',
    entry_point='gym_aero.mt_envs:RandomWaypointMTEnv',
)
register(
    id='LandMT-v0',
    entry_point='gym_aero.mt_envs:LandMTEnv',
)
register(
    id='PerchMT-v0',
    entry_point='gym_aero.mt_envs:PerchMTEnv',
)


# PID Envs
register(
    id='PIDTest-v0',
    entry_point='gym_aero.pid_envs:PIDTestEnv',
)

register(
    id='PIDHover-v0',
    entry_point='gym_aero.pid_envs:PIDHoverEnv',
)
register(
    id='PIDRandomWaypointFH-v0',
    entry_point='gym_aero.pid_envs:PIDRandomWaypointFHEnv',
)
register(
    id='PIDRandomWaypointNH-v0',
    entry_point='gym_aero.pid_envs:PIDRandomWaypointNHEnv',
)
register(
    id='PIDLand-v0',
    entry_point='gym_aero.pid_envs:PIDLandEnv',
)

# comparison envs
register(
    id='HoverComparison-v0',
    entry_point='gym_aero.comparison:HoverComparison',
)

register(
    id='HoverOld-v0',
    entry_point='gym_aero.comparison:HoverOld',
)