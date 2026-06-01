"""
Visualization and rendering helpers for robotics simulations.

Wraps Polyscope GUI for robot state visualization including
body poses, joint states, and contact points.
"""

import numpy as np


class RobotViewer:
    def __init__(self, solver, config, output_dir=None):
        self.solver = solver
        self.config = config
        self.output_dir = output_dir

    def show(self):
        try:
            from lcs_gui import SimulationGUI
            gui = SimulationGUI(self.solver.solver, self.config, self.output_dir)
            gui.show()
        except ImportError:
            print("[robot_viewer] lcs_gui not available, running headless")

    def log_joint_state(self):
        from robotics.utils.joint_utils import log_joint_states
        return log_joint_states(self.solver.solver)
