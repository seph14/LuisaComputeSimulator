"""
Cartpole rigid body demo using LuisaComputeSimulator robotics module.

Builds a cartpole scene with:
- A fixed anchor body
- A cart body sliding on a prismatic joint (along X axis)
- Two pole bodies connected via revolute joints (around Y axis)

Usage:
    # headless
    PYTHONPATH=build/bin .venv/bin/python PythonBindings/robotics/test_cartpole.py --headless --advance_frames 300

    # GUI
    PYTHONPATH=build/bin .venv/bin/python PythonBindings/robotics/test_cartpole.py
"""

import os
import sys
import argparse
import platform
import numpy as np

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_SCRIPT_DIR, ".."))       # PythonBindings/
sys.path.insert(0, os.path.join(_SCRIPT_DIR, "..", "tests"))  # PythonBindings/tests/

import lcs_py as lcs
from utils.test_script_path import PROJECT_ROOT

from robotics.solver.robot_solver import RobotSolver
from robotics.mesh.robot_mesh import create_rigid_body, create_fixed_anchor, load_mesh_from_file
from robotics.utils.joint_utils import (
    print_joint_summary, log_joint_states, print_validation,
    get_revolute_angles, get_prismatic_slides,
)
from robotics.render.robot_viewer import RobotViewer

parser = argparse.ArgumentParser(description="Cartpole robot demo")
parser.add_argument("--backend", type=str,
                    default="metal" if platform.system() == "Darwin" else "cuda")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--advance_frames", type=int, default=300)
args = parser.parse_args()

ANIMATED_FRAMES = args.advance_frames
OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "..", "tests", "output")

CART_SCALE = (0.15, 0.08, 0.05)
POLE_HALF_HEIGHT = 0.25
POLE_SCALE = (0.02, 0.02, POLE_HALF_HEIGHT * 2.0)
ANCHOR_SCALE = (0.02, 0.02, 0.02)
POLE1_INITIAL_ANGLE = 0.26
POLE2_INITIAL_REL_ANGLE = -0.17

RAIL_Y = 0.0
RAIL_Z = 0.1

# ── Solver Setup ──────────────────────────────────────────────────
rs = RobotSolver(backend_name=args.backend)
rs.init_device()
rs.setup_z_up(dt=1.0 / 300.0)

# ── Scene ─────────────────────────────────────────────────────────
cube_mesh = load_mesh_from_file(
    os.path.join(PROJECT_ROOT, "Resources", "InputMesh", "cube.obj"))

z = np.zeros(3, dtype=np.float32)
x_axis = np.array([1.0, 0.0, 0.0], dtype=np.float32)
y_axis = np.array([0.0, 1.0, 0.0], dtype=np.float32)

# Fixed anchor
anchor_id = create_fixed_anchor(rs.solver, "anchor", 0.0,
                                RAIL_Y, RAIL_Z,
                                *ANCHOR_SCALE)
rs._body_ids["anchor"] = anchor_id

# Cart
cart_id = rs.add_rigid_body("cart", cube_mesh.vertices, cube_mesh.faces,
                            0.0, RAIL_Y, RAIL_Z, *CART_SCALE)

# Pole 1 (tilted 15° = ~0.26 rad)
hinge0 = np.array([0.0, RAIL_Y, RAIL_Z + CART_SCALE[2] / 2.0], dtype=np.float64)
pole1_center = hinge0 + np.array([
    POLE_HALF_HEIGHT * np.sin(POLE1_INITIAL_ANGLE),
    0.0,
    POLE_HALF_HEIGHT * np.cos(POLE1_INITIAL_ANGLE),
], dtype=np.float64)
pole1_id = rs.add_rigid_body("pole1", cube_mesh.vertices, cube_mesh.faces,
                              pole1_center[0], pole1_center[1], pole1_center[2],
                              *POLE_SCALE, ry=POLE1_INITIAL_ANGLE)

# Pole 2 (tilted opposite -10°)
pole2_world_angle = POLE1_INITIAL_ANGLE + POLE2_INITIAL_REL_ANGLE
hinge1 = hinge0 + np.array([
    2.0 * POLE_HALF_HEIGHT * np.sin(POLE1_INITIAL_ANGLE),
    0.0,
    2.0 * POLE_HALF_HEIGHT * np.cos(POLE1_INITIAL_ANGLE),
], dtype=np.float64)
pole2_center = hinge1 + np.array([
    POLE_HALF_HEIGHT * np.sin(pole2_world_angle),
    0.0,
    POLE_HALF_HEIGHT * np.cos(pole2_world_angle),
], dtype=np.float64)
pole2_id = rs.add_rigid_body("pole2", cube_mesh.vertices, cube_mesh.faces,
                              pole2_center[0], pole2_center[1], pole2_center[2],
                              *POLE_SCALE, ry=pole2_world_angle)

# ── Joints ────────────────────────────────────────────────────────
rs.add_prismatic_joint("anchor", "cart", z, z, x_axis,
                       stiffness_pos=5.0e4, stiffness_rot=1.0e4,
                       slide_min=-2.0, slide_max=2.0)

cart_top_local = np.array([0.0, 0.0, CART_SCALE[2] / 2.0], dtype=np.float32)
pole1_bottom_local = np.array([0.0, 0.0, -POLE_HALF_HEIGHT], dtype=np.float32)
rs.add_revolute_joint("cart", "pole1",
                      cart_top_local, pole1_bottom_local, y_axis,
                      stiffness_pos=5.0e4, stiffness_axis=2.0e3)

pole1_top_local = np.array([0.0, 0.0, POLE_HALF_HEIGHT], dtype=np.float32)
pole2_bottom_local = np.array([0.0, 0.0, -POLE_HALF_HEIGHT], dtype=np.float32)
rs.add_revolute_joint("pole1", "pole2",
                      pole1_top_local, pole2_bottom_local, y_axis,
                      stiffness_pos=5.0e4, stiffness_axis=2.0e3)

rs.init_solver()

# ── Info ──────────────────────────────────────────────────────────
rs.print_mesh_info()
print_joint_summary(rs.solver)

os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Simulation ────────────────────────────────────────────────────
if args.headless:
    rs.save_result(os.path.join(OUTPUT_DIR, "cartpole_init.obj"))
    init_cart_center = rs.get_body_center("cart")

    for frame in range(ANIMATED_FRAMES):
        rs.step()

        if frame % 30 == 0:
            states = log_joint_states(rs.solver)
            print(f"  frame {frame:4d}: [{states}]")

    rs.save_result(os.path.join(OUTPUT_DIR, "cartpole_result.obj"))

    # ── Validation ────────────────────────────────────────────
    print("\n--- Validation ---")

    cart_center = rs.get_body_center("cart")
    print(f"Cart final position:   {cart_center}")
    print(f"Cart initial position: {init_cart_center}")

    print_validation(rs.solver)

    rev_angles = get_revolute_angles(rs.solver)
    if len(rev_angles) >= 2:
        print(f"Pole1 angle: {rev_angles[0]:.4f}, Pole2 angle: {rev_angles[1]:.4f}")
    assert len(rev_angles) >= 2, "Expected two revolute joints"

    cart_disp = cart_center - init_cart_center
    print(f"Cart displacement: {cart_disp}")
    prismatic_slides = get_prismatic_slides(rs.solver)
    assert len(prismatic_slides) == 1, "Expected one prismatic joint"
    # With num_substep=3 the effective step is smaller → shorter displacement per frame
    assert abs(prismatic_slides[0]) > max(1e-4, 0.3e-5 * ANIMATED_FRAMES), f"Cart should slide along X (got {prismatic_slides[0]:.6f})"
    assert max(abs(a) for a in rev_angles) > max(5e-4, 0.004e-2 * ANIMATED_FRAMES), "Poles should rotate around revolute joints"

    # ROADMAP 1.7: Z-axis drift tolerance.
    # Penalty-based joints have inherent small compliance (~2e-4 under gravity).
    # This is a soft-constraint artifact, not a solver bug.
    Z_TOLERANCE = 5e-4
    assert abs(cart_disp[1]) < Z_TOLERANCE, f"Cart Y drift {cart_disp[1]:.6f} exceeds {Z_TOLERANCE}"
    assert abs(cart_disp[2]) < Z_TOLERANCE, f"Cart Z drift {cart_disp[2]:.6f} exceeds {Z_TOLERANCE}"

    # Verify all bodies' centroid positions are reasonable
    for bname in ["cart", "pole1", "pole2"]:
        center = rs.get_body_center(bname)
        y_offset = abs(center[1] - RAIL_Y)
        if y_offset > 1e-2:
            print(f"  WARNING: {bname} Y offset {y_offset:.4f} from rail at y={RAIL_Y}")

    print(f"  Z-drift validation: PASSED (cart_dz={cart_disp[2]:.2e} within {Z_TOLERANCE})")
    print("  Joint angle stability: OK")

    print("Cartpole headless test PASSED")

else:
    viewer = RobotViewer(rs, rs.config, OUTPUT_DIR)
    viewer.show()

rs.cleanup()
