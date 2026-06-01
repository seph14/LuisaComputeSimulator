"""
LuisaComputeSimulator Robotics Module.

Submodules:
- solver:   Robot-specific NewtonSolver wrapper
- mesh:     Rigid body and collision mesh helpers
- render:   Visualization utilities
- training: RL environment abstractions
- utils:    Joint query, physics math utilities
"""

from robotics import solver
from robotics import mesh
from robotics import render
from robotics import training
from robotics import utils
