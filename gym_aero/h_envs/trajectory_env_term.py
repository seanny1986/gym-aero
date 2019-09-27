from math import pi, sin, cos, acos, exp
import gym
from gym import error, spaces, utils
from gym.utils import seeding
from gym_aero.envs import trajectory_env
import simulation.animation_gl as ani_gl

class TermTrajectoryEnv(trajectory_env.TrajectoryEnv):

    def __init__(self):
        super(TermTrajectoryEnv, self).__init__()

    def term_reward(self, state):
        xyz, _, _, _, _ = state
        u = sum([(x-g)**2 for x, g in zip(xyz, self.goal_xyz)])**0.5
        v = sum([(x-g)**2 for x, g in zip(xyz, self.goal_xyz_next)])**0.5
        return -15.*u**2-u*v

    def step(self, action, term):
        self.t += 1
        action = self.translate_action(action)
        xyz, zeta, xyz_dot, pqr = super(trajectory_env.TrajectoryEnv, self).step(action)
        sin_zeta = [sin(z) for z in zeta]
        cos_zeta = [cos(z) for z in zeta]
        curr_rpm = self.get_rpm()
        normalized_rpm = [rpm/self.max_rpm for rpm in curr_rpm]
        reward, info = self.reward(xyz, sin_zeta, cos_zeta, xyz_dot, pqr, action)
        term_rew = self.term_reward((xyz, sin_zeta, cos_zeta, xyz_dot, pqr)) if term == 1 else 0.
        if term == 1: self.next_goal()
        done = self.terminal((xyz, zeta, xyz_dot, pqr))
        obs = self.get_state_obs((xyz, sin_zeta, cos_zeta, xyz_dot, pqr), action, normalized_rpm)
        info.update({"term_rew" : term_rew})
        return obs, reward, done, info