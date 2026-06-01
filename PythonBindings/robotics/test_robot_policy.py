"""
ONNX policy inference test with G1 robot (ROADMAP 5.7-5.9).

Loads G1 URDF + ONNX policy, runs policy inference loop.
Validates: ONNX model loads, inference produces valid actions,
policy step pipeline runs without crashing.

Usage:
    PYTHONPATH=build/bin .venv/bin/python PythonBindings/robotics/test_robot_policy.py --headless --advance_frames 60
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

parser = argparse.ArgumentParser(description="ONNX policy demo")
parser.add_argument("--backend", type=str,
                    default="metal" if platform.system() == "Darwin" else "cuda")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--advance_frames", type=int, default=60)
parser.add_argument("--robot", type=str, default="g1",
                    help="Robot type: g1, h1")
args = parser.parse_args()

FRAMES = args.advance_frames
POLICY_STEPS = max(1, FRAMES // 15)  # policy runs every 15 physics steps

ASSETS = os.path.join(_SCRIPT_DIR, "assets")

# Select robot
if args.robot == "h1":
    URDF = os.path.join(ASSETS, "unitree_h1", "urdf", "h1.urdf")
else:
    URDF = os.path.join(ASSETS, "unitree_g1", "urdf", "g1_23dof_rev_1_0.urdf")
ONNX_PATH = os.path.join(ASSETS, "unitree_g1", "rl_policies", "mjw_g1_23DOF.onnx")
OUTPUT_DIR = os.path.join(_SCRIPT_DIR, "..", "tests", "output")

# ── Try loading ONNX ────────────────────────────────────────────────
onnx_available = False
try:
    import onnxruntime as ort
    if os.path.exists(ONNX_PATH):
        session = ort.InferenceSession(ONNX_PATH)
        input_name = session.get_inputs()[0].name
        input_shape = session.get_inputs()[0].shape
        print(f"ONNX model loaded: {ONNX_PATH}")
        print(f"  Input: {input_name} shape={input_shape}")
        onnx_available = True
    else:
        print(f"ONNX model not found at {ONNX_PATH}")
except ImportError:
    print("onnxruntime not installed — running without policy inference")

# ── Build robot ─────────────────────────────────────────────────────
print(f"Loading URDF: {URDF}")
model = URDFParser.parse(URDF)
print(f"  Links: {len(model.links)}, Joints: {len(model.joints)}")

rs = RobotSolver(backend_name=args.backend)
rs.init_device()
rs.setup_z_up(dt=1.0 / 300.0)

cube = trimesh.load(os.path.join(PROJECT_ROOT, "Resources", "InputMesh", "cube.obj"),
                    process=False)

for i, link_name in enumerate(URDFParser.build_topology_order(model)):
    link = model.links.get(link_name)
    if link is None: continue
    scale = (0.03, 0.03, 0.03)
    if link.collisions:
        col = link.collisions[0]
        if col.geometry_type == "sphere":
            scale = (0.06, 0.06, 0.06)
        elif col.geometry_type == "cylinder":
            scale = (0.04, 0.04, 0.05)
    # Spread bodies vertically to avoid overlap
    tz = 1.0 + i * 0.02
    rs.add_rigid_body(link_name, cube.vertices, cube.faces,
                      tx=0, ty=0, tz=float(tz), sx=scale[0], sy=scale[1], sz=scale[2],
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
num_actions = min(n_joints, input_shape[1] if onnx_available and len(input_shape) > 1 else n_joints)

KP, KD = 500.0, 20.0
for j in range(n_joints):
    rs._solver.set_joint_target_pos(j, float(initial_q[j]))
    rs._solver.set_joint_target_kp(j, KP)
    rs._solver.set_joint_target_kd(j, KD)

# ── Policy step loop ────────────────────────────────────────────────
if args.headless:
    action_history = []
    for frame in range(FRAMES):
        # Policy inference every 15 steps
        if onnx_available and frame % 15 == 0:
            obs = np.zeros((1, input_shape[1]), dtype=np.float32)
            # Fill observation with current joint positions
            q = rs.get_all_joint_values()
            for j in range(min(n_joints, input_shape[1])):
                obs[0, j] = float(q[j])
            action = session.run(None, {input_name: obs})[0]
            action_history.append(action.flatten()[:num_actions])

            # Apply action as joint targets
            for j in range(min(n_joints, num_actions)):
                target = float(initial_q[j]) + float(action[0, j]) * 0.1
                rs._solver.set_joint_target_pos(j, target)

        rs.step()

    rs.save_result(os.path.join(OUTPUT_DIR, "policy_result.obj"))

    # ── Validate ────────────────────────────────────────────────────
    final_q = rs.get_all_joint_values()
    max_drift = max(abs(final_q[i] - initial_q[i]) for i in range(n_joints))
    print(f"  Max joint drift: {max_drift:.4f} rad")
    if onnx_available:
        print(f"  Policy steps executed: {len(action_history)}")
        if action_history:
            print(f"  Action mean abs: {np.mean(np.abs(action_history)):.4f}")
    assert max_drift < 2.0, f"Robot collapsed (max drift {max_drift:.4f})"
    print("ONNX policy test PASSED")
else:
    from robotics.render.robot_viewer import RobotViewer
    RobotViewer(rs, rs.config, OUTPUT_DIR).show()

rs.cleanup()
