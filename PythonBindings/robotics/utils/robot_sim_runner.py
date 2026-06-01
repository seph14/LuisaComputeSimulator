"""
Standard simulation step-cycle templates for robotics simulations.

Provides reusable simulate() and simulate_with_render() functions
that can be shared across all robot demo scripts.
"""

import os
import numpy as np


def simulate(solver, num_frames, callback=None, log_interval=30):
    """
    Standard headless simulation loop.

    Args:
        solver: RobotSolver instance (must already have init_solver() called).
        num_frames: Number of physics steps to advance.
        callback: Optional callable(frame_idx, solver) called after each step.
        log_interval: Print joint state every N frames (0 to disable).

    Returns:
        dict with final joint values and body centers.
    """
    for frame in range(num_frames):
        solver.step()

        if callback is not None:
            callback(frame, solver)

        if log_interval > 0 and frame % log_interval == 0:
            from robotics.utils.joint_utils import log_joint_states
            states = log_joint_states(solver.solver)
            print(f"  frame {frame:4d}: [{states}]")

    # Collect final state
    result = {
        'joint_values': solver.get_all_joint_values(),
        'joint_types': solver.get_all_joint_types(),
    }
    return result


def simulate_with_render(solver, config, output_dir=None):
    """
    Interactive simulation loop with Polyscope GUI.

    Args:
        solver: RobotSolver instance.
        config: SceneParams from solver.get_config().
        output_dir: Optional directory for saving output meshes.
    """
    try:
        from lcs_gui import SimulationGUI
        gui = SimulationGUI(solver.solver, config, output_dir)
        gui.show()
    except ImportError:
        print("[robot_sim_runner] lcs_gui not available, falling back to headless")
        simulate(solver, num_frames=300)


def save_init_and_result(solver, output_dir, prefix="scene"):
    """
    Save initial and final OBJ snapshots of the simulation.

    Args:
        solver: RobotSolver instance.
        output_dir: Directory to save OBJ files.
        prefix: Filename prefix.
    """
    os.makedirs(output_dir, exist_ok=True)
    solver.save_result(os.path.join(output_dir, f"{prefix}_init.obj"))


def validate_z_drift(body_centers, tolerance=1e-4):
    """
    Validate that all bodies stay in the XY plane (Z drift < tolerance).

    Args:
        body_centers: dict of body_name -> np.ndarray[3] of initial center positions.
        tolerance: Maximum allowed Z-axis drift.

    Returns:
        True if all bodies pass, False otherwise.
    """
    for name, center in body_centers.items():
        if abs(center[2]) > tolerance:
            print(f"  FAIL: {name} Z={center[2]:.6f} exceeds tolerance {tolerance}")
            return False
    return True


def validate_joint_angle_limits(solver, angle_limits, tolerance=0.01):
    """
    Validate that revolute joint angles stay within specified limits.

    Args:
        solver: RobotSolver instance.
        angle_limits: dict of joint_idx -> (lower, upper) in radians.
        tolerance: Allowed overshoot beyond limits.

    Returns:
        True if all joints pass, False otherwise.
    """
    from robotics.utils.joint_utils import get_revolute_angles
    angles = get_revolute_angles(solver.solver)
    all_pass = True
    for joint_idx, (lo, hi) in angle_limits.items():
        if joint_idx < len(angles):
            a = angles[joint_idx]
            if a < lo - tolerance or a > hi + tolerance:
                print(f"  FAIL: Joint {joint_idx} angle {a:.4f} outside [{lo}, {hi}]")
                all_pass = False
    return all_pass
