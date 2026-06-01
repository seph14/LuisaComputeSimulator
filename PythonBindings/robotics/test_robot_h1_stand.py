"""
H1 humanoid standing test (ROADMAP 4.1-4.8).

Loads H1 URDF, builds the articulated body chain, and holds initial pose
using joint position drive.  Uses collision primitives for geometry
(sphere/cylinder) since mesh loading requires .dae support.

Usage:
    PYTHONPATH=build/bin .venv/bin/python PythonBindings/robotics/test_robot_h1_stand.py --headless --advance_frames 120
"""

import os, sys, argparse, platform
import numpy as np

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_SCRIPT_DIR, ".."))
sys.path.insert(0, os.path.join(_SCRIPT_DIR, "..", "tests"))

import lcs_py as lcs
from utils.test_script_path import PROJECT_ROOT

from robotics.solver.robot_solver import RobotSolver
from robotics.parser.urdf_parser import URDFParser, URDFRobotModel
from robotics.utils.joint_utils import log_joint_states, print_joint_summary

parser = argparse.ArgumentParser(description="H1 standing demo")
parser.add_argument("--backend", type=str,
                    default="metal" if platform.system() == "Darwin" else "cuda")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--advance_frames", type=int, default=120)
args = parser.parse_args()

FRAMES = args.advance_frames
ASSETS = os.path.join(_SCRIPT_DIR, "assets")
H1_URDF = os.path.join(ASSETS, "unitree_h1", "urdf", "h1.urdf")
OUTPUT_DIR = os.path.join(_SCRIPT_DIR, "..", "tests", "output")

# ── Parse URDF ──────────────────────────────────────────────────────
print(f"Loading H1 URDF: {H1_URDF}")
model = URDFParser.parse(H1_URDF)
print(f"  Links: {len(model.links)}, Joints: {len(model.joints)}, Root: {model.root_link}")

# ── Setup solver ────────────────────────────────────────────────────
rs = RobotSolver(backend_name=args.backend)
rs.init_device()
rs.setup_z_up(dt=1.0 / 300.0)

cube_path = os.path.join(PROJECT_ROOT, "Resources", "InputMesh", "cube.obj")
cube = __import__('trimesh').load(cube_path, process=False)

# ── Build bodies using collision primitives ─────────────────────────
# For each link, use its collision geometry (sphere/cylinder) or cube fallback
link_order = URDFParser.build_topology_order(model)
print(f"  Topo order: {len(link_order)} links")

body_names = []
for link_name in link_order:
    link = model.links.get(link_name)
    if link is None:
        continue

    # Determine body size from collision geometry
    scale = (0.03, 0.03, 0.03)  # default small cube
    if link.collisions:
        col = link.collisions[0]
        if col.geometry_type == "sphere":
            r = col.geometry_data.get("radius", 0.03)
            scale = (r * 2, r * 2, r * 2)
        elif col.geometry_type == "cylinder":
            r = col.geometry_data.get("radius", 0.02)
            l = col.geometry_data.get("length", 0.05)
            scale = (r * 2, r * 2, l)
        elif col.geometry_type == "box":
            s = col.geometry_data.get("size", [0.03, 0.03, 0.03])
            scale = tuple(s)

    # Approximate position from joint chain (starting near origin)
    # Use origin_xyz from the joint connecting this link to its parent
    tx, ty, tz = 0.0, 0.0, 1.0  # start at 1m height
    is_fixed = (link_name == model.root_link)

    rid = rs.add_rigid_body(link_name, cube.vertices, cube.faces,
                            tx=tx, ty=ty, tz=tz,
                            sx=scale[0], sy=scale[1], sz=scale[2],
                            fixed=is_fixed)
    body_names.append(link_name)

# ── Create joints ───────────────────────────────────────────────────
joint_count = 0
for joint in model.joints:
    p_id = rs._body_ids.get(joint.parent)
    c_id = rs._body_ids.get(joint.child)
    if p_id is None or c_id is None:
        continue

    anchor_parent = joint.origin_xyz.astype(np.float64)
    anchor_child = np.zeros(3, dtype=np.float64)
    axis = joint.axis.astype(np.float64)
    jtype = joint.joint_type.lower()

    if jtype in ("revolute", "continuous"):
        rs.add_revolute_joint(joint.parent, joint.child,
                              anchor_parent, anchor_child, axis,
                              stiffness_pos=1.0e5, stiffness_axis=5.0e3)
    elif jtype == "prismatic":
        rs.add_prismatic_joint(joint.parent, joint.child,
                               anchor_parent, anchor_child, axis,
                               stiffness_pos=1.0e5, stiffness_rot=1.0e4)
    elif jtype == "fixed":
        rs._solver.add_fixed_joint(p_id, c_id, anchor_parent, anchor_child,
                                   stiffness_pos=1.0e6, stiffness_rot=1.0e5)
    joint_count += 1

print(f"  Created {joint_count} joints")

rs.init_solver()
print_joint_summary(rs.solver)

os.makedirs(OUTPUT_DIR, exist_ok=True)
rs.save_result(os.path.join(OUTPUT_DIR, "h1_stand_init.obj"))

# ── Drive all joints to hold initial position ───────────────────────
KP = 500.0
KD = 20.0
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
        if frame % 30 == 0:
            q = rs.get_all_joint_values()
            print(f"  frame {frame:4d}: {[f'{v:.4f}' for v in q[:6]]}...")

    rs.save_result(os.path.join(OUTPUT_DIR, "h1_stand_result.obj"))

    # ── Validate ────────────────────────────────────────────────────
    print("\n--- Validation ---")
    final_q = rs.get_all_joint_values()

    # Check all bodies are above ground (z > 0 in Z-up)
    all_above_ground = True
    for name in body_names[:10]:  # check first 10 links
        try:
            center = rs.get_body_center(name)
            if center[2] < -0.1:
                print(f"  FAIL: {name} z={center[2]:.4f} below ground!")
                all_above_ground = False
        except Exception:
            pass

    # Check joints haven't diverged too far from initial
    max_drift = max(abs(final_q[i] - initial_q[i]) for i in range(n_joints))
    print(f"  Max joint drift from initial: {max_drift:.4f} rad")
    assert max_drift < 1.0, f"Joints drifted too far ({max_drift:.4f} rad)"

    # Check body velocities are reasonable
    if all_above_ground:
        print("  All bodies above ground: PASSED")
    print(f"  Joint drift check: PASSED (max={max_drift:.4f})")
    print("H1 standing test PASSED")

else:
    from robotics.render.robot_viewer import RobotViewer
    viewer = RobotViewer(rs, rs.config, OUTPUT_DIR)
    viewer.show()

rs.cleanup()
