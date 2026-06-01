"""
Robot environment abstractions for RL training loops.

Provides base classes and utilities for building
physics-based reinforcement learning environments.
"""


class RobotEnv:
    def __init__(self, robot_solver):
        self._solver = robot_solver

    @property
    def solver(self):
        return self._solver

    def reset(self):
        raise NotImplementedError

    def step(self, action):
        raise NotImplementedError

    def get_observation(self):
        raise NotImplementedError

    def get_reward(self):
        raise NotImplementedError

    def is_done(self):
        return False
