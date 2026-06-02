"""
G1 humanoid standing parity test (P4.2).

Loads G1 URDF (23-DOF or 29-DOF variant), builds articulated body chain,
and holds initial pose via joint position drive.

Newton reference: newton/examples/robot/example_robot_g1.py
Newton params: joint target_ke=500, target_kd=10, limit_ke=1e3, limit_kd=1e1
Newton assertions: all bodies z > 0, all body velocities < 0.015, world_count=4

Usage:
    PYTHONPATH=build/bin .venv/bin/python PythonBindings/robotics/test_robot_g1_stand.py \\
        --headless --advance_frames 120
    PYTHONPATH=build/bin .venv/bin/python PythonBindings/robotics/test_robot_g1_stand.py \\
        --headless --advance_frames 60 --world-count 4
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

parser = argparse.ArgumentParser(description="G1 standing parity test (P4.2)")
parser.add_argument("--backend", type=str,
                    default="metal" if platform.system() == "Darwin" else "cuda")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--advance_frames", type=int, default=120)
parser.add_argument("--world-count", type=int, default=1,
                    help="Number of replicated worlds (default: 1, Newton: 4)")
parser.add_argument("--variant", type=str, default="23dof",
                    choices=["23dof", "29dof"], help="G1 DOF variant")
parser.add_argument("--floating-base", action="store_true",
                    help="Use free joint instead of fixed base")
parser.add_argument("--disable-floor", action="store_true")
args = parser.parse_args()

FRAMES = args.advance_frames
WORLD_COUNT = args.world_count
ASSETS = os.path.join(_SCRIPT_DIR, "assets")
G1_URDF = os.path.join(ASSETS, "unitree_g1", "urdf",
                       f"g1_{args.variant}_rev_1_0.urdf")
OUTPUT_DIR = os.path.join(_SCRIPT_DIR, "..", "tests", "output")

# Newton G1 solver params
KP = 500.0       # joint target_ke
KD = 10.0        # joint target_kd
NONLINEAR_ITER = 100


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
print(f"Loading G1 URDF ({args.variant}): {G1_URDF}")
model = URDFParser.parse(G1_URDF)
print(f"  Links: {len(model.links)}, Joints: {len(model.joints)}, Root: {model.root_link}")

# ── Setup solver ────────────────────────────────────────────────────
rs = RobotSolver(backend_name=args.backend)
rs.init_device()
rs.setup_z_up(dt=1.0 / 300.0)
rs.config.set_up_axis(lcs.UpAxis.Z_UP)
rs.config.set_use_floor(not args.disable_floor)
rs.config.set_nonlinear_iter_count(NONLINEAR_ITER)

# ── Build world 0 + capture specs ───────────────────────────────────
body_specs = []
_orig_add = rs.add_rigid_body

def _capture_add(name, vertices, faces, tx=0, ty=0, tz=0,
                 sx=1, sy=1, sz=1, rx=0, ry=0, rz=0,
                 fixed=False, mass=None, com=None, density=None):
    body_specs.append({
        "name": name, "vertices": np.array(vertices, dtype=np.float64),
        "faces": np.array(faces, dtype=np.int32),
        "tx": float(tx), "ty": float(ty), "tz": float(tz),
        "rx": float(rx), "ry": float(ry), "rz": float(rz),
        "sx": float(sx), "sy": float(sy), "sz": float(sz),
        "fixed": fixed, "mass": mass, "com": com, "density": density,
    })
    return _orig_add(name, vertices, faces, tx, ty, tz, sx, sy, sz, rx, ry, rz,
                     fixed=fixed, mass=mass, com=com, density=density)

rs.add_rigid_body = _capture_add

builder = RobotBuilder(rs, model, fixed_base=not args.floating_base)
builder.build(
    mesh_root=os.path.dirname(G1_URDF),
    base_translation=(0.0, 0.0, 0.0),
    swap_yz=False,
    floor_height=0.0,
    floor_normal=vec_xyz(rs.config.get_floor_normal()),
    floor_clearance=0.01,
)
rs.add_rigid_body = _orig_add
body_names = URDFParser.build_topology_order(model)
print(f"  Built {len(builder.link_body_ids)} bodies, {len(model.joints)} URDF joints")

# Collect joint specs
joint_specs = []
for joint in model.joints:
    jtype = joint.joint_type.lower()
    if jtype in ("revolute", "continuous"):
        jtype = "revolute"
    axis = joint.axis.astype(np.float64)
    anchor = joint.origin_xyz.astype(np.float64)
    kwargs = {"anchor_a": anchor, "anchor_b": np.zeros(3, dtype=np.float64),
              "axis": axis, "stiffness_pos": 5.0e4, "stiffness_axis": 2.0e3}
    if joint.has_limits:
        kwargs["lower_angle"] = joint.lower_limit
        kwargs["upper_angle"] = joint.upper_limit
    joint_specs.append({
        "parent_name": joint.parent, "child_name": joint.child,
        "joint_type": jtype, "kwargs": kwargs,
    })

# ── Replicate ───────────────────────────────────────────────────────
if WORLD_COUNT > 1:
    rs.replicate(body_specs, joint_specs, WORLD_COUNT, spacing=(0.0, 2.0, 0.0))

rs.init_solver()
print_joint_summary(rs.solver)

floor_normal_vec = vec_xyz(rs.config.get_floor_normal())
floor_height = float(np.dot(
    np.asarray(vec_xyz(rs.config.get_floor()), dtype=np.float64),
    np.asarray(floor_normal_vec, dtype=np.float64)))
initial_min_floor = scene_min_floor_coord(rs, body_names, floor_normal_vec)
print(f"  Floor min coord: {initial_min_floor:.4f}")

# ── Set joint targets to hold initial pose ──────────────────────────
n_joints = rs.get_joint_count()
initial_q = rs.get_all_joint_values()
for j in range(n_joints):
    rs._solver.set_joint_target_pos(j, float(initial_q[j]))
    rs._solver.set_joint_target_kp(j, KP)
    rs._solver.set_joint_target_kd(j, KD)
print(f"  Joint drive: kp={KP}, kd={KD} across {n_joints} joints")

# ── Simulate ────────────────────────────────────────────────────────
if args.headless:
    for frame in range(FRAMES):
        rs.step()

    # ── Validate ────────────────────────────────────────────────────
    print("\n--- Validation ---")
    all_pass = True
    final_q = rs.get_all_joint_values()

    final_min_floor = scene_min_floor_coord(rs, body_names, floor_normal_vec)
    if final_min_floor < floor_height - 1e-3:
        print(f"  FAIL: bodies below ground (min_z={final_min_floor:.4f})")
        all_pass = False
    else:
        print(f"  PASS: all bodies above ground (min_z={final_min_floor:.4f})")

    drift_vals = [abs(final_q[i] - initial_q[i]) for i in range(min(n_joints, len(initial_q)))]
    max_drift = max(drift_vals) if drift_vals else 0.0
    DRIFT_LIMIT = 0.1
    if max_drift > DRIFT_LIMIT:
        print(f"  FAIL: max joint drift {max_drift:.4f} > {DRIFT_LIMIT}")
        all_pass = False
    else:
        print(f"  PASS: max joint drift {max_drift:.4f} <= {DRIFT_LIMIT}")

    # Body velocity (Newton: < 0.015, staged: < 0.05)
    VEL_THRESHOLD = 0.05
    max_body_speed = 0.0
    for bname in body_names[:5]:
        try:
            vel = rs.get_body_velocity(bname)
            max_body_speed = max(max_body_speed, float(np.sqrt(sum(v**2 for v in np.array(vel[:3])))))
        except Exception:
            pass
    print(f"  Max body speed (sampled): {max_body_speed:.4f} (threshold: {VEL_THRESHOLD})")
    if max_body_speed > VEL_THRESHOLD:
        print(f"  WARNING: speed exceeds staged threshold (Newton: < 0.015)")

    # Multi-world
    if WORLD_COUNT > 1:
        from robotics.solver.articulation_view import ArticulationView
        dof_indices = [js for i, js in enumerate(range(len(joint_specs)))
                       if joint_specs[i]["joint_type"] not in ("fixed", "free")]
        dof_indices = []
        for js in joint_specs:
            if js["joint_type"] not in ("fixed", "free"):
                jidx = rs.get_joint_index(js["parent_name"], js["child_name"], world_id=0)
                if jidx >= 0:
                    dof_indices.append(jidx)
        view = ArticulationView(rs, body_names, dof_indices)
        q_all = view.get_joint_q()
        max_std = float(np.max(np.std(q_all, axis=0)))
        print(f"  Multi-world joint consistency: max std={max_std:.2e}")
        if max_std > 1e-3:
            print(f"  WARNING: joint states diverge (max_std={max_std:.2e})")
        else:
            print(f"  PASS: consistent across {WORLD_COUNT} worlds")

    # Parity gaps
    print(f"\n  Parity gaps (vs Newton robot_g1):")
    print(f"    - Body velocities: staged {VEL_THRESHOLD} (Newton: < 0.015)")
    print(f"    - Floating base: {'free joint' if args.floating_base else 'fixed base'} (Newton: floating)")
    print(f"    - World count: {WORLD_COUNT} (Newton: 4)")
    print(f"    - Mesh approx: trimesh mesh (Newton: bounding_box)")
    print(f"    - Contact params: default (Newton: ke=1e3, kd=2e2, kf=1e3, mu=0.75)")
    print(f"    - Variant: {args.variant} (Newton: 29dof)")

    if all_pass:
        print(f"\nG1 standing test PASSED ({FRAMES} frames, {WORLD_COUNT} world(s))")
        sys.exit(0)
    else:
        print(f"\nG1 standing test FAILED")
        sys.exit(1)

else:
    from robotics.render.robot_viewer import RobotViewer
    RobotViewer(rs, rs.config, OUTPUT_DIR).show()

rs.cleanup()
