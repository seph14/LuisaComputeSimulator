"""
Allegro hand test (ROADMAP 6.1-6.4).

Builds a 16-DOF multi-fingered hand with revolute joints from
procedural geometry, validates joint stability under gravity.

Usage:
    PYTHONPATH=build/bin .venv/bin/python PythonBindings/robotics/test_robot_allegro.py --headless --advance_frames 120
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

parser = argparse.ArgumentParser(description="Allegro hand demo")
parser.add_argument("--backend", type=str,
                    default="metal" if platform.system() == "Darwin" else "cuda")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--advance_frames", type=int, default=120)
args = parser.parse_args()

FRAMES = args.advance_frames
OUTPUT_DIR = os.path.join(_SCRIPT_DIR, "..", "tests", "output")

rs = RobotSolver(backend_name=args.backend)
rs.init_device()
rs.setup_z_up(dt=1.0 / 300.0)

cube = trimesh.load(os.path.join(PROJECT_ROOT, "Resources", "InputMesh", "cube.obj"),
                    process=False)

# ── Build Allegro-like hand ─────────────────────────────────────────
# Palm (base) + 4 fingers × 4 links = 17 bodies, 16 revolute joints
FINGERS = ["index", "middle", "ring", "thumb"]
FINGER_Y = [0.03, 0.01, -0.01, -0.03]
LINK_LENGTHS = [0.04, 0.03, 0.025, 0.02]

# Palm (fixed base)
rs.add_rigid_body("palm", cube.vertices, cube.faces,
                  tx=0, ty=0, tz=0.5, sx=0.08, sy=0.06, sz=0.03, fixed=True)

axis = np.array([0.0, 1.0, 0.0], dtype=np.float64)  # Y-axis revolute

total_joints = 0
for fi, (fname, fy) in enumerate(zip(FINGERS, FINGER_Y)):
    prev_body = "palm"
    prev_anchor_z = 0.015  # top of palm
    z_pos = 0.53

    for li, ll in enumerate(LINK_LENGTHS):
        link_name = f"{fname}_{li}"
        rs.add_rigid_body(link_name, cube.vertices, cube.faces,
                          tx=0, ty=fy, tz=z_pos + ll / 2,
                          sx=0.012, sy=0.012, sz=ll)

        anchor_p = np.array([0.0, fy, prev_anchor_z], dtype=np.float64)
        anchor_c = np.array([0.0, 0.0, -ll / 2], dtype=np.float64)
        rs.add_revolute_joint(prev_body, link_name, anchor_p, anchor_c, axis,
                              stiffness_pos=5.0e4, stiffness_axis=3.0e3)
        total_joints += 1
        prev_body = link_name
        prev_anchor_z = z_pos + ll
        z_pos += ll

print(f"  Finger joints: {total_joints} (expected 16)")

rs.init_solver()
print_joint_summary(rs.solver)
os.makedirs(OUTPUT_DIR, exist_ok=True)

n_joints = rs.get_joint_count()
initial_q = rs.get_all_joint_values()

KP, KD = 300.0, 15.0
for j in range(n_joints):
    rs._solver.set_joint_target_pos(j, float(initial_q[j]))
    rs._solver.set_joint_target_kp(j, KP)
    rs._solver.set_joint_target_kd(j, KD)

if args.headless:
    for f in range(FRAMES):
        rs.step()
    rs.save_result(os.path.join(OUTPUT_DIR, "allegro_result.obj"))
    final_q = rs.get_all_joint_values()
    max_drift = max(abs(final_q[i] - initial_q[i]) for i in range(n_joints))
    print(f"  Max joint drift: {max_drift:.4f} rad")
    assert max_drift < 0.5, f"Hand collapsed ({max_drift:.4f})"
    assert n_joints == 16, f"Expected 16 joints, got {n_joints}"

    # Check all fingertips stay above ground
    for fname in FINGERS:
        c = rs.get_body_center(f"{fname}_3")
        assert c[2] > 0.1, f"{fname} finger too low: z={c[2]:.4f}"

    print("Allegro hand test PASSED")
else:
    from robotics.render.robot_viewer import RobotViewer
    RobotViewer(rs, rs.config, OUTPUT_DIR).show()
rs.cleanup()
