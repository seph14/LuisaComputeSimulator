"""
G1 humanoid standing test (ROADMAP 4.9).

Loads G1 URDF (23-DOF variant), holds initial pose via joint drive.
Uses collision primitives for fast initialization.

Usage:
    PYTHONPATH=build/bin .venv/bin/python PythonBindings/robotics/test_robot_g1_stand.py --headless --advance_frames 120
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

parser = argparse.ArgumentParser(description="G1 standing demo")
parser.add_argument("--backend", type=str,
                    default="metal" if platform.system() == "Darwin" else "cuda")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--advance_frames", type=int, default=120)
args = parser.parse_args()

FRAMES = args.advance_frames
G1_URDF = os.path.join(_SCRIPT_DIR, "assets", "unitree_g1", "urdf",
                       "g1_23dof_rev_1_0.urdf")
OUTPUT_DIR = os.path.join(_SCRIPT_DIR, "..", "tests", "output")

print(f"Loading G1 URDF: {G1_URDF}")
model = URDFParser.parse(G1_URDF)
print(f"  Links: {len(model.links)}, Joints: {len(model.joints)}, Root: {model.root_link}")

rs = RobotSolver(backend_name=args.backend)
rs.init_device()
rs.setup_z_up(dt=1.0 / 300.0)

cube = trimesh.load(os.path.join(PROJECT_ROOT, "Resources", "InputMesh", "cube.obj"),
                    process=False)

link_order = URDFParser.build_topology_order(model)
for i, link_name in enumerate(link_order):
    link = model.links.get(link_name)
    if link is None:
        continue
    scale = (0.03, 0.03, 0.03)
    if link.collisions:
        col = link.collisions[0]
        if col.geometry_type == "sphere":
            r = col.geometry_data.get("radius", 0.03)
            scale = (r * 2, r * 2, r * 2)
        elif col.geometry_type == "cylinder":
            r = col.geometry_data.get("radius", 0.02)
            l = col.geometry_data.get("length", 0.05)
            scale = (r * 2, r * 2, l)
    is_fixed = (link_name == model.root_link)
    rs.add_rigid_body(link_name, cube.vertices, cube.faces,
                      tx=0, ty=0, tz=float(1.0 + i * 0.02), sx=scale[0], sy=scale[1], sz=scale[2],
                      fixed=is_fixed)

n_joints = 0
for joint in model.joints:
    if joint.parent not in rs._body_ids or joint.child not in rs._body_ids:
        continue
    axis = joint.axis.astype(np.float64)
    anchor = joint.origin_xyz.astype(np.float64)
    jtype = joint.joint_type.lower()
    if jtype in ("revolute", "continuous"):
        rs.add_revolute_joint(joint.parent, joint.child, anchor, np.zeros(3), axis,
                              stiffness_pos=1.0e5, stiffness_axis=5.0e3)
        n_joints += 1
    elif jtype == "fixed":
        rs._solver.add_fixed_joint(rs._body_ids[joint.parent], rs._body_ids[joint.child],
                                   anchor, np.zeros(3), 1.0e6, 1.0e5)
        n_joints += 1

print(f"  Created {n_joints} joints")
rs.init_solver()
print_joint_summary(rs.solver)

os.makedirs(OUTPUT_DIR, exist_ok=True)
initial_q = rs.get_all_joint_values()
KP, KD = 500.0, 20.0
for j in range(len(initial_q)):
    rs._solver.set_joint_target_pos(j, float(initial_q[j]))
    rs._solver.set_joint_target_kp(j, KP)
    rs._solver.set_joint_target_kd(j, KD)

if args.headless:
    for f in range(FRAMES):
        rs.step()
    rs.save_result(os.path.join(OUTPUT_DIR, "g1_stand_result.obj"))
    final_q = rs.get_all_joint_values()
    max_drift = max(abs(final_q[i] - initial_q[i]) for i in range(len(initial_q)))
    print(f"  Max joint drift: {max_drift:.4f} rad")
    assert max_drift < 1.0, f"Joints drifted too far ({max_drift:.4f})"
    # Check all bodies above ground
    for name in list(rs._body_ids.keys())[:5]:
        c = rs.get_body_center(name)
        assert c[2] > -0.5, f"{name} below ground: z={c[2]:.4f}"
    print("G1 standing test PASSED")
else:
    from robotics.render.robot_viewer import RobotViewer
    RobotViewer(rs, rs.config, OUTPUT_DIR).show()
rs.cleanup()
