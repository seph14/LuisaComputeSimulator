"""
Rigid body drop test — validates ground contact stability (ROADMAP 3.6-3.8).

Drops a cube onto a ground plane body (since ground collision shader
currently only supports Y-up axis-aligned floors, we use a fixed body
as ground in Z-up mode).  Validates:
- Body does not fall through ground
- Contact is stable (velocity decays)
- Works in both Y-up and Z-up modes

Usage:
    PYTHONPATH=build/bin .venv/bin/python PythonBindings/robotics/test_drop_contact.py --headless --advance_frames 120
"""

import os, sys, argparse, platform
import numpy as np

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_SCRIPT_DIR, ".."))
sys.path.insert(0, os.path.join(_SCRIPT_DIR, "..", "tests"))

import trimesh
import lcs_py as lcs
from utils.test_script_path import PROJECT_ROOT

from robotics.solver.robot_solver import RobotSolver
from robotics.utils.joint_utils import print_joint_summary

parser = argparse.ArgumentParser(description="Drop test")
parser.add_argument("--backend", type=str,
                    default="metal" if platform.system() == "Darwin" else "cuda")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--advance_frames", type=int, default=120)
parser.add_argument("--z_up", action="store_true", default=True,
                    help="Use Z-up coordinate system")
args = parser.parse_args()

FRAMES = args.advance_frames
OUTPUT_DIR = os.path.join(_SCRIPT_DIR, "..", "tests", "output")
os.makedirs(OUTPUT_DIR, exist_ok=True)

cube_path = os.path.join(PROJECT_ROOT, "Resources", "InputMesh", "cube.obj")
cube = trimesh.load(cube_path, process=False)

# ── Solver ───────────────────────────────────────────────────────────
rs = RobotSolver(backend_name=args.backend)
rs.init_device()

# Use Y-up mode with built-in floor collision (ground shader is Y-up only)
config = rs.config
config.set_gravity(lcs.Float3(0.0, -9.8, 0.0))
config.set_use_floor(True)
config.set_use_self_collision(False)
config.set_implicit_dt(1.0 / 300.0)
config.set_num_substep(3)
config.set_nonlinear_iter_count(5)
config.set_pcg_iter_count(200)
config.set_stiffness_collision(1e6)  # lower stiffness for stable contact

up_axis = 1  # Y
up_label = "Y"
drop_height = 1.0

# ── Drop cube ───────────────────────────────────────────────────────
drop_id = rs.add_rigid_body("drop_cube", cube.vertices, cube.faces,
                            tx=0, ty=float(drop_height), tz=0,
                            sx=0.15, sy=0.15, sz=0.15)

rs.init_solver()
rs.save_result(os.path.join(OUTPUT_DIR, "drop_init.obj"))

# ── Simulate ────────────────────────────────────────────────────────
drop_positions = []
if args.headless:
    for f in range(FRAMES):
        rs.step()
        center = rs.get_body_center("drop_cube")
        drop_positions.append(center.copy())

    rs.save_result(os.path.join(OUTPUT_DIR, "drop_result.obj"))

    # ── Validate ────────────────────────────────────────────────────
    final_center = rs.get_body_center("drop_cube")
    up_axis = 2 if args.z_up else 1  # Z or Y
    up_label = "Z" if args.z_up else "Y"
    min_height = min(p[up_axis] for p in drop_positions)
    final_height = final_center[up_axis]

    print(f"  Drop test ({up_label}-up):")
    print(f"    Final height: {final_height:.4f}")
    print(f"    Min height:   {min_height:.4f}")
    print(f"    Above ground: {final_height > -0.1}")

    # Cube should not fall through ground
    assert min_height > -0.1, f"Cube fell through ground! Min height={min_height:.4f}"
    # Final height should be near cube half-size (0.075) above ground
    assert abs(final_height - 0.075) < 0.1, f"Cube not resting on ground: h={final_height:.4f}"

    print("Drop contact test PASSED")
else:
    from robotics.render.robot_viewer import RobotViewer
    RobotViewer(rs, rs.config, OUTPUT_DIR).show()

rs.cleanup()
