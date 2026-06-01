"""
Franka Emika Panda arm test (ROADMAP 7.1-7.4).

Loads Franka URDF, builds 7-DOF arm, validates joint chain stability.
Uses collision primitives from URDF.

Usage:
    PYTHONPATH=build/bin .venv/bin/python PythonBindings/robotics/test_robot_franka.py --headless --advance_frames 120
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
from robotics.parser.urdf_parser import URDFParser
from robotics.utils.joint_utils import print_joint_summary

parser = argparse.ArgumentParser(description="Franka Panda arm demo")
parser.add_argument("--backend", type=str,
                    default="metal" if platform.system() == "Darwin" else "cuda")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--advance_frames", type=int, default=120)
args = parser.parse_args()

FRAMES = args.advance_frames
FRANKA_URDF = os.path.join(_SCRIPT_DIR, "assets", "franka_emika_panda",
                           "urdf", "fr3.urdf")
OUTPUT_DIR = os.path.join(_SCRIPT_DIR, "..", "tests", "output")

print(f"Loading Franka URDF: {FRANKA_URDF}")
model = URDFParser.parse(FRANKA_URDF)
print(f"  Links: {len(model.links)}, Joints: {len(model.joints)}, Root: {model.root_link}")

rs = RobotSolver(backend_name=args.backend)
rs.init_device()
rs.setup_z_up(dt=1.0 / 300.0)

cube = trimesh.load(os.path.join(PROJECT_ROOT, "Resources", "InputMesh", "cube.obj"),
                    process=False)

for i, link_name in enumerate(URDFParser.build_topology_order(model)):
    link = model.links.get(link_name)
    if link is None: continue
    scale = (0.04, 0.04, 0.04)
    if link.collisions:
        col = link.collisions[0]
        if col.geometry_type == "sphere":
            scale = (0.06, 0.06, 0.06)
        elif col.geometry_type == "cylinder":
            scale = (0.04, 0.04, 0.08)
        elif col.geometry_type == "box":
            sz = col.geometry_data.get("size", [0.04, 0.04, 0.04])
            scale = tuple(sz)
    rs.add_rigid_body(link_name, cube.vertices, cube.faces,
                      tx=0, ty=0, tz=float(0.5 + i * 0.08),
                      sx=scale[0], sy=scale[1], sz=scale[2],
                      fixed=(link_name == model.root_link))

for joint in model.joints:
    if joint.parent not in rs._body_ids or joint.child not in rs._body_ids:
        continue
    axis = joint.axis.astype(np.float64)
    anchor = joint.origin_xyz.astype(np.float64)
    jt = joint.joint_type.lower()
    if jt in ("revolute", "continuous"):
        rs.add_revolute_joint(joint.parent, joint.child, anchor, np.zeros(3), axis,
                              stiffness_pos=1.0e5, stiffness_axis=5.0e3)
    elif jt == "fixed":
        rs._solver.add_fixed_joint(rs._body_ids[joint.parent], rs._body_ids[joint.child],
                                   anchor, np.zeros(3), 1.0e6, 1.0e5)

rs.init_solver()
print_joint_summary(rs.solver)
os.makedirs(OUTPUT_DIR, exist_ok=True)

n_joints = rs.get_joint_count()
initial_q = rs.get_all_joint_values()
print(f"  Revolute joints: {n_joints} (expected 7)")

KP, KD = 300.0, 15.0
for j in range(n_joints):
    rs._solver.set_joint_target_pos(j, float(initial_q[j]))
    rs._solver.set_joint_target_kp(j, KP)
    rs._solver.set_joint_target_kd(j, KD)

if args.headless:
    for f in range(FRAMES):
        rs.step()
    final_q = rs.get_all_joint_values()
    max_drift = max(abs(final_q[i] - initial_q[i]) for i in range(n_joints))
    print(f"  Max joint drift: {max_drift:.4f} rad")
    assert n_joints >= 6, f"Expected >=6 joints, got {n_joints}"
    assert max_drift < 1.0, f"Arm collapsed ({max_drift:.4f})"
    print("Franka Panda test PASSED")
else:
    from robotics.render.robot_viewer import RobotViewer
    RobotViewer(rs, rs.config, OUTPUT_DIR).show()
rs.cleanup()
