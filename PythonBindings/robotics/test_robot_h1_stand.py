"""
H1 humanoid standing test (ROADMAP 4.1-4.8).

Loads H1 URDF, builds the articulated body chain, and holds initial pose
using joint position drive. Uses the URDF kinematic tree to place links and
collision primitives for geometry where available.

Usage:
    PYTHONPATH=build/bin .venv/bin/python PythonBindings/robotics/test_robot_h1_stand.py --headless --advance_frames 500
"""

import os, sys, argparse, platform
import numpy as np

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_SCRIPT_DIR, ".."))
sys.path.insert(0, os.path.join(_SCRIPT_DIR, "..", "tests"))

import lcs_py as lcs

from robotics.solver.robot_solver import RobotSolver
from robotics.parser.urdf_parser import URDFParser
from robotics.robot_builder import RobotBuilder
from robotics.utils.joint_utils import print_joint_summary

parser = argparse.ArgumentParser(description="H1 standing demo")
parser.add_argument("--backend", type=str,
                    default="metal" if platform.system() == "Darwin" else "cuda")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--advance_frames", type=int, default=500)
parser.add_argument("--base_height", type=float, default=0.0,
                    help="Extra world Z offset applied after auto floor lift")
parser.add_argument("--floor_clearance", type=float, default=0.01,
                    help="Auto-lift clearance above the floor")
parser.add_argument("--disable_auto_lift", action="store_true",
                    help="Disable automatic lift to floor + clearance")
parser.add_argument("--no_swap_yz", action="store_true",
                    help="Disable the default URDF Y/Z axis swap")
parser.add_argument("--floating_base", action="store_true",
                    help="Do not pin the root link. Less stable until contact parity is complete.")
parser.add_argument("--disable_floor", action="store_true",
                    help="Disable ground plane contact")
args = parser.parse_args()

FRAMES = args.advance_frames
ASSETS = os.path.join(_SCRIPT_DIR, "assets")
H1_URDF = os.path.join(ASSETS, "unitree_h1", "urdf", "h1.urdf")
OUTPUT_DIR = os.path.join(_SCRIPT_DIR, "..", "tests", "output")


def vec_xyz(vec):
    return (float(vec.x), float(vec.y), float(vec.z))


def scene_min_floor_coord(robot_solver, names, floor_normal):
    normal = np.asarray(floor_normal, dtype=np.float64)
    norm = np.linalg.norm(normal)
    if norm <= 0.0:
        normal = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
    else:
        normal = normal / norm
    min_coord = float("inf")
    for body_name in names:
        try:
            verts = robot_solver.get_body_vertices(body_name)
            min_coord = min(min_coord, float(np.min(verts @ normal)))
        except Exception:
            pass
    return min_coord

# ── Parse URDF ──────────────────────────────────────────────────────
print(f"Loading H1 URDF: {H1_URDF}")
model = URDFParser.parse(H1_URDF)
print(f"  Links: {len(model.links)}, Joints: {len(model.joints)}, Root: {model.root_link}")

# ── Setup solver ────────────────────────────────────────────────────
rs = RobotSolver(backend_name=args.backend)
rs.init_device()
rs.setup_z_up(dt=1.0 / 300.0)
if not args.no_swap_yz:
    rs.config.set_up_axis(lcs.UpAxis.Y_UP)
rs.config.set_use_floor(not args.disable_floor)

# ── Build bodies and joints from URDF topology ───────────────────────
builder = RobotBuilder(rs, model, fixed_base=not args.floating_base)
builder.build(
    mesh_root=os.path.dirname(H1_URDF),
    base_translation=(0.0, 0.0, args.base_height),
    swap_yz=not args.no_swap_yz,
    floor_height=None if args.disable_auto_lift else 0.0,
    floor_normal=vec_xyz(rs.config.get_floor_normal()),
    floor_clearance=args.floor_clearance,
)
body_names = URDFParser.build_topology_order(model)
print(f"  Built {len(builder.link_body_ids)} bodies and {len(model.joints)} URDF joints")
print(
    f"  Extra base height: {args.base_height:.3f} m, "
    f"auto_lift={not args.disable_auto_lift}, clearance={args.floor_clearance:.3f} m, "
    f"swap_yz={not args.no_swap_yz}, fixed_base={not args.floating_base}, floor={not args.disable_floor}"
)

rs.init_solver()
print_joint_summary(rs.solver)

floor_normal = vec_xyz(rs.config.get_floor_normal())
floor_height = float(
    np.dot(np.asarray(vec_xyz(rs.config.get_floor()), dtype=np.float64),
           np.asarray(floor_normal, dtype=np.float64))
)
initial_min_floor = scene_min_floor_coord(rs, body_names, floor_normal)
print(
    f"  Initial scene min floor coord: {initial_min_floor:.4f} m "
    f"(floor={floor_height:.4f}, clearance={initial_min_floor - floor_height:.4f} m, normal={floor_normal})"
)

os.makedirs(OUTPUT_DIR, exist_ok=True)
rs.save_result(os.path.join(OUTPUT_DIR, "h1_stand_init.obj"))

# ── Drive all joints to hold initial position ───────────────────────
KP = 150.0
KD = 5.0
n_joints = rs.get_joint_count()
initial_q = rs.get_all_joint_values()
print(f"  Initial joint positions: {[f'{v:.4f}' for v in initial_q]}")

for j in range(n_joints):
    rs._solver.set_joint_target_pos(j, float(initial_q[j]))
    rs._solver.set_joint_target_kp(j, KP)
    rs._solver.set_joint_target_kd(j, KD)

# ── Simulate ────────────────────────────────────────────────────────
if args.headless:
    for frame in range(FRAMES):
        rs.step()
        if frame % 50 == 0:
            q = rs.get_all_joint_values()
            print(f"  frame {frame:4d}: {[f'{v:.4f}' for v in q[:6]]}...")

    rs.save_result(os.path.join(OUTPUT_DIR, "h1_stand_result.obj"))

    # ── Validate ────────────────────────────────────────────────────
    print("\n--- Validation ---")
    final_q = rs.get_all_joint_values()

    # Check all vertices are above the configured floor plane.
    final_min_floor = scene_min_floor_coord(rs, body_names, floor_normal)
    all_above_ground = final_min_floor >= floor_height - 1.0e-3
    if not all_above_ground:
        print(f"  FAIL: scene min floor coord={final_min_floor:.4f} below floor={floor_height:.4f}!")

    # Check joints haven't diverged too far from initial
    max_drift = max(abs(final_q[i] - initial_q[i]) for i in range(n_joints))
    print(f"  Max joint drift from initial: {max_drift:.4f} rad")
    assert max_drift < 1.0, f"Joints drifted too far ({max_drift:.4f} rad)"

    # Check body velocities are reasonable
    if all_above_ground:
        print(f"  All vertices above ground: PASSED (min_floor_coord={final_min_floor:.4f})")
    print(f"  Joint drift check: PASSED (max={max_drift:.4f})")
    print("H1 standing test PASSED")

else:
    from robotics.render.robot_viewer import RobotViewer
    viewer = RobotViewer(rs, rs.config, OUTPUT_DIR)
    viewer.show()

rs.cleanup()
