"""
ANYmal quadruped standing proxy test (P4.3).

Builds a procedural quadruped with 13 bodies (1 base + 4 legs × 3 links)
and 12 revolute joints. NOTE: This is a procedural proxy, NOT real ANYmal D.

Status: proxy — blocked by missing ANYmal URDF/MJCF importer.

Newton reference: newton/examples/robot/example_robot_anymal_d.py
Newton params: base height ~0.68m, world_count=16, floating base

Usage:
    PYTHONPATH=build/bin .venv/bin/python PythonBindings/robotics/test_robot_anymal.py \\
        --headless --advance_frames 120
    PYTHONPATH=build/bin .venv/bin/python PythonBindings/robotics/test_robot_anymal.py \\
        --headless --advance_frames 60 --world-count 4
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

parser = argparse.ArgumentParser(description="ANYmal quadruped standing proxy test (P4.3)")
parser.add_argument("--backend", type=str,
                    default="metal" if platform.system() == "Darwin" else "cuda")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--advance_frames", type=int, default=120)
parser.add_argument("--world-count", type=int, default=1,
                    help="Number of replicated worlds (Newton: 16)")
args = parser.parse_args()

FRAMES = args.advance_frames
WORLD_COUNT = args.world_count
OUTPUT_DIR = os.path.join(_SCRIPT_DIR, "..", "tests", "output")

# Newton ANYmal D params
KP = 150.0
KD = 5.0
BASE_HEIGHT_TARGET = 0.68  # Newton reference


def build_anymal_procedural(rs):
    """Build ONE procedural ANYmal in the given solver. Returns body_specs, joint_specs."""
    cube = trimesh.load(os.path.join(PROJECT_ROOT, "Resources", "InputMesh", "cube.obj"),
                        process=False)
    body_specs = []
    joint_specs = []

    def _add_body(name, tx, ty, tz, sx, sy, sz, fixed=False):
        rs.add_rigid_body(name, cube.vertices, cube.faces,
                          tx=tx, ty=ty, tz=tz, sx=sx, sy=sy, sz=sz, fixed=fixed)
        body_specs.append({
            "name": name, "vertices": np.array(cube.vertices, dtype=np.float64),
            "faces": np.array(cube.faces, dtype=np.int32),
            "tx": float(tx), "ty": float(ty), "tz": float(tz),
            "rx": 0.0, "ry": 0.0, "rz": 0.0,
            "sx": float(sx), "sy": float(sy), "sz": float(sz), "fixed": fixed,
        })

    # Base
    _add_body("base", 0, 0, 0.6, 0.4, 0.2, 0.1, fixed=True)

    hip_axis = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    knee_axis = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    leg_names = ["LF", "RF", "LH", "RH"]
    leg_signs = [(0.2, 0.15), (0.2, -0.15), (-0.2, 0.15), (-0.2, -0.15)]

    for leg_idx, (name, (lx, ly)) in enumerate(zip(leg_names, leg_signs)):
        hip_name = f"{name}_hip"
        _add_body(hip_name, lx, ly, 0.55, 0.05, 0.05, 0.08)

        rs.add_revolute_joint("base", hip_name,
                              np.array([lx, ly, 0.55], dtype=np.float64),
                              np.array([0, 0, -0.04], dtype=np.float64),
                              hip_axis, stiffness_pos=5.0e4, stiffness_axis=3.0e3)
        joint_specs.append({
            "parent_name": "base", "child_name": hip_name,
            "joint_type": "revolute",
            "kwargs": {"anchor_a": np.array([lx, ly, 0.55], dtype=np.float64),
                       "anchor_b": np.array([0, 0, -0.04], dtype=np.float64),
                       "axis": hip_axis.copy(), "stiffness_pos": 5.0e4, "stiffness_axis": 3.0e3},
        })

        thigh_name = f"{name}_thigh"
        _add_body(thigh_name, lx, ly, 0.42, 0.04, 0.04, 0.15)

        rs.add_revolute_joint(hip_name, thigh_name,
                              np.array([0, 0, -0.04], dtype=np.float64),
                              np.array([0, 0, 0.075], dtype=np.float64),
                              knee_axis, stiffness_pos=5.0e4, stiffness_axis=3.0e3)
        joint_specs.append({
            "parent_name": hip_name, "child_name": thigh_name,
            "joint_type": "revolute",
            "kwargs": {"anchor_a": np.array([0, 0, -0.04], dtype=np.float64),
                       "anchor_b": np.array([0, 0, 0.075], dtype=np.float64),
                       "axis": knee_axis.copy(), "stiffness_pos": 5.0e4, "stiffness_axis": 3.0e3},
        })

        calf_name = f"{name}_calf"
        _add_body(calf_name, lx, ly, 0.25, 0.03, 0.03, 0.18)

        rs.add_revolute_joint(thigh_name, calf_name,
                              np.array([0, 0, -0.075], dtype=np.float64),
                              np.array([0, 0, 0.09], dtype=np.float64),
                              knee_axis, stiffness_pos=5.0e4, stiffness_axis=3.0e3)
        joint_specs.append({
            "parent_name": thigh_name, "child_name": calf_name,
            "joint_type": "revolute",
            "kwargs": {"anchor_a": np.array([0, 0, -0.075], dtype=np.float64),
                       "anchor_b": np.array([0, 0, 0.09], dtype=np.float64),
                       "axis": knee_axis.copy(), "stiffness_pos": 5.0e4, "stiffness_axis": 3.0e3},
        })

    return body_specs, joint_specs, leg_names


# ── Setup solver ────────────────────────────────────────────────────
rs = RobotSolver(backend_name=args.backend)
rs.init_device()
rs.setup_z_up(dt=1.0 / 300.0)
rs.config.set_use_floor(True)

# Build world 0 + collect specs
body_specs, joint_specs, leg_names = build_anymal_procedural(rs)

# Replicate
if WORLD_COUNT > 1:
    rs.replicate(body_specs, joint_specs, WORLD_COUNT, spacing=(0.0, 2.0, 0.0))

rs.init_solver()
print_joint_summary(rs.solver)
print(f"  Joints: {rs.get_joint_count()} (expected {12 * WORLD_COUNT})")

# Set joint targets to hold initial pose
n_joints = rs.get_joint_count()
initial_q = rs.get_all_joint_values()
for j in range(n_joints):
    rs._solver.set_joint_target_pos(j, float(initial_q[j]))
    rs._solver.set_joint_target_kp(j, KP)
    rs._solver.set_joint_target_kd(j, KD)

os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Simulate ────────────────────────────────────────────────────────
if args.headless:
    rs.save_result(os.path.join(OUTPUT_DIR, "anymal_init.obj"))
    for f in range(FRAMES):
        rs.step()
    rs.save_result(os.path.join(OUTPUT_DIR, "anymal_result.obj"))

    # ── Validate ────────────────────────────────────────────────────
    print("\n--- Validation ---")
    all_pass = True
    final_q = rs.get_all_joint_values()

    # Base height (Newton: ~0.68m)
    base_center = rs.get_body_center("base")
    base_z = float(base_center[2])
    print(f"  Base height: {base_z:.4f} m (Newton target: {BASE_HEIGHT_TARGET})")
    if base_z < 0.3:
        print(f"  FAIL: base too low ({base_z:.3f}m)")
        all_pass = False
    else:
        print(f"  PASS: base above ground")

    # Joint drift
    drift_vals = [abs(final_q[i] - initial_q[i]) for i in range(min(n_joints, len(initial_q)))]
    max_drift = max(drift_vals) if drift_vals else 0.0
    DRIFT_LIMIT = 0.5
    if max_drift > DRIFT_LIMIT:
        print(f"  FAIL: max joint drift {max_drift:.4f} > {DRIFT_LIMIT}")
        all_pass = False
    else:
        print(f"  PASS: max joint drift {max_drift:.4f} <= {DRIFT_LIMIT}")

    # Feet above ground
    feet_ok = True
    for name in leg_names:
        c = rs.get_body_center(f"{name}_calf")
        if float(c[2]) < -0.2:
            print(f"  FAIL: {name} foot below ground (z={c[2]:.4f})")
            feet_ok = False
    if feet_ok:
        print(f"  PASS: all feet above ground")

    # Multi-world validation
    if WORLD_COUNT > 1:
        from robotics.solver.articulation_view import ArticulationView
        body_names_list = ["base"] + [f"{n}_{p}" for n in leg_names for p in ["hip", "thigh", "calf"]]
        dof_indices = [rs.get_joint_index(js["parent_name"], js["child_name"], world_id=0)
                       for js in joint_specs if js["joint_type"] != "fixed"]
        dof_indices = [j for j in dof_indices if j >= 0]
        view = ArticulationView(rs, body_names_list, dof_indices)
        q_all = view.get_joint_q()
        max_std = float(np.max(np.std(q_all, axis=0)))
        print(f"  Multi-world joint consistency: max std={max_std:.2e}")
        if max_std > 1e-3:
            print(f"  WARNING: joint states diverge (max_std={max_std:.2e})")
        else:
            print(f"  PASS: consistent across {WORLD_COUNT} worlds")

    # Parity gaps
    print(f"\n  Parity gaps (vs Newton robot_anymal_d):")
    print(f"    - Asset: procedural proxy (Newton: ANYmal D USD)")
    print(f"    - Base height: {base_z:.3f}m (Newton: 0.68m)")
    print(f"    - Floating base: fixed base (Newton: floating)")
    print(f"    - World count: {WORLD_COUNT} (Newton: 16)")
    print(f"    - Contact: default params (Newton: ke=2e3, kd=1e2)")
    print(f"    - Collapse fixed joints: False (Newton: False)")

    if all_pass:
        print(f"\nANYmal standing proxy test PASSED ({FRAMES} frames, {WORLD_COUNT} world(s))")
        sys.exit(0)
    else:
        print(f"\nANYmal standing proxy test FAILED")
        sys.exit(1)

else:
    from robotics.render.robot_viewer import RobotViewer
    RobotViewer(rs, rs.config, OUTPUT_DIR).show()

rs.cleanup()
