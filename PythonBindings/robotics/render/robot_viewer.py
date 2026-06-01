"""
Visualization and rendering helpers for robotics simulations.

Wraps Polyscope GUI for robot state visualization including
body poses, joint states, and contact points.
"""

import numpy as np


class RobotViewer:
    """Visualization wrapper for robotics simulations (ROADMAP B.6).

    Supports Polyscope GUI integration, body pose tracking, joint state
    display, and contact point visualization stubs.
    """

    def __init__(self, solver, config, output_dir=None):
        self.solver = solver
        self.config = config
        self.output_dir = output_dir
        self._body_colors: dict = {}  # body_name -> RGB color
        self._joint_highlight: list = []  # joint indices to highlight

    def show(self):
        """Launch interactive Polyscope GUI."""
        try:
            from lcs_gui import SimulationGUI
            gui = SimulationGUI(self.solver.solver, self.config, self.output_dir)
            gui.show()
        except ImportError:
            print("[robot_viewer] lcs_gui not available, running headless")

    def log_joint_state(self):
        """Get formatted joint state string."""
        from robotics.utils.joint_utils import log_joint_states
        return log_joint_states(self.solver.solver)

    def set_body_color(self, body_name: str, color=(0.8, 0.2, 0.2)):
        """Assign a display color to a body (RGB 0-1)."""
        self._body_colors[body_name] = color

    def highlight_joint(self, joint_idx: int):
        """Mark a joint for visual highlighting."""
        if joint_idx not in self._joint_highlight:
            self._joint_highlight.append(joint_idx)

    def get_body_poses(self, body_names: list = None):
        """Get world-frame poses for specified bodies.

        Returns dict: body_name -> (translation_xyz, rotation_quaternion_wxyz)
        """
        names = body_names or list(self.solver._body_ids.keys())
        poses = {}
        for name in names:
            if name in self.solver._body_ids:
                poses[name] = self.solver.get_body_pose(name)
        return poses

    def print_joint_summary(self):
        """Print a summary of current joint states."""
        from robotics.utils.joint_utils import print_joint_summary, print_validation
        print_joint_summary(self.solver.solver)
        print_validation(self.solver.solver)
