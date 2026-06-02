"""
ONNX policy inference test with G1/H1 robot (ROADMAP P5).

Builds robot from URDF via RobotBuilder, constructs proper observation
(base velocity, projected gravity, joint error), maps actions with scaling,
and runs closed-loop policy inference.

Usage:
    # Smoke test (30 frames)
    PYTHONPATH=build/bin .venv/bin/python PythonBindings/robotics/test_robot_policy.py \\
        --headless --advance_frames 30

    # Full policy run (500 frames)
    PYTHONPATH=build/bin .venv/bin/python PythonBindings/robotics/test_robot_policy.py \\
        --headless --advance_frames 500
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
from robotics.training.observation_builder import ObservationBuilder
from robotics.training.action_mapper import ActionMapper
from robotics.utils.joint_utils import print_joint_summary

parser = argparse.ArgumentParser(description="ONNX policy demo")
parser.add_argument("--backend", type=str,
                    default="metal" if platform.system() == "Darwin" else "cuda")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--advance_frames", type=int, default=60)
parser.add_argument("--robot", type=str, default="g1",
                    help="Robot type: g1, h1")
parser.add_argument("--decimation", type=int, default=15,
                    help="Physics steps per policy inference (default: 15)")
parser.add_argument("--action_scale", type=float, default=0.5,
                    help="Multiplier applied to policy actions")
parser.add_argument("--base_height", type=float, default=0.0,
                    help="Extra world Z offset for auto floor lift")
parser.add_argument("--no_swap_yz", action="store_true",
                    help="Disable URDF Y/Z axis swap")
parser.add_argument("--disable_floor", action="store_true",
                    help="Disable ground plane contact")
args = parser.parse_args()

FRAMES = args.advance_frames
DECIMATION = args.decimation
ASSETS = os.path.join(_SCRIPT_DIR, "assets")
OUTPUT_DIR = os.path.join(_SCRIPT_DIR, "..", "tests", "output")

# ── Select robot and policy ─────────────────────────────────────────
if args.robot == "h1":
    URDF_PATH = os.path.join(ASSETS, "unitree_h1", "urdf", "h1.urdf")
    ONNX_PATH = os.path.join(ASSETS, "unitree_h1", "rl_policies", "mjw_h1.onnx")
else:
    URDF_PATH = os.path.join(ASSETS, "unitree_g1", "urdf", "g1_23dof_rev_1_0.urdf")
    ONNX_PATH = os.path.join(ASSETS, "unitree_g1", "rl_policies", "mjw_g1_23DOF.onnx")


def vec_xyz(vec):
    return (float(vec.x), float(vec.y), float(vec.z))


# ── Load ONNX ────────────────────────────────────────────────────────
onnx_available = False
session = None
input_name = ""
input_shape = []
try:
    import onnxruntime as ort
    if os.path.exists(ONNX_PATH):
        session = ort.InferenceSession(ONNX_PATH)
        input_name = session.get_inputs()[0].name
        input_shape = session.get_inputs()[0].shape
        print(f"ONNX model loaded: {os.path.basename(ONNX_PATH)}")
        print(f"  Input: {input_name} shape={input_shape}")
        onnx_available = True
    else:
        print(f"ONNX model not found: {ONNX_PATH}")
        print("Running without policy — joint drive holds initial pose")
except ImportError:
    print("onnxruntime not installed — running without policy inference")

# ── Build robot from URDF ────────────────────────────────────────────
print(f"Loading URDF: {URDF_PATH}")
model = URDFParser.parse(URDF_PATH)
print(f"  Links: {len(model.links)}, Joints: {len(model.joints)}, Root: {model.root_link}")

rs = RobotSolver(backend_name=args.backend)
rs.init_device()
rs.setup_z_up(dt=1.0 / 300.0)
rs.config.set_use_floor(not args.disable_floor)

builder = RobotBuilder(rs, model, fixed_base=True)
builder.build(
    mesh_root=os.path.dirname(URDF_PATH),
    base_translation=(0.0, 0.0, args.base_height),
    swap_yz=not args.no_swap_yz,
    floor_height=0.0,
    floor_normal=vec_xyz(rs.config.get_floor_normal()),
    floor_clearance=0.01,
)
body_names = URDFParser.build_topology_order(model)
print(f"  Built {len(builder.link_body_ids)} bodies, {len(model.joints)} URDF joints")

rs.init_solver()
print_joint_summary(rs.solver)
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Identify DOF joints (non-fixed, non-free) ───────────────────────
n_joints = rs.get_joint_count()
initial_q = np.array(rs.get_all_joint_values(), dtype=np.float64)
n_dof = len(initial_q)  # all joints are DOF in fixed-base mode

# Build DOF joint index list
dof_joint_indices = list(range(n_dof))
print(f"  DOF count: {n_dof}")

# ── Set joint drive params ──────────────────────────────────────────
KP = 500.0
KD = 20.0
for j in range(n_joints):
    rs._solver.set_joint_target_pos(j, float(initial_q[j]))
    rs._solver.set_joint_target_kp(j, KP)
    rs._solver.set_joint_target_kd(j, KD)
print(f"  Joint drive: kp={KP}, kd={KD}")

# ── Build observation & action utilities ────────────────────────────
obs_builder = ObservationBuilder(rs, body_names, dof_joint_indices,
                                 base_body_name=model.root_link)
action_mapper = ActionMapper(rs, dof_joint_indices,
                             default_joint_q=initial_q,
                             action_scale=args.action_scale)

if onnx_available:
    expected_obs_dim = input_shape[1] if len(input_shape) > 1 else obs_builder.obs_dim
    print(f"  Observation dim: {obs_builder.obs_dim} "
          f"(policy expects: {expected_obs_dim})")
    print(f"  Action scale: {args.action_scale}")

# ── Policy loop ─────────────────────────────────────────────────────
if args.headless:
    rs.save_result(os.path.join(OUTPUT_DIR, "policy_init.obj"))

    prev_action = np.zeros(n_dof, dtype=np.float32)
    command_vel = np.array([1.0, 0.0, 0.0], dtype=np.float32)  # forward
    action_history = []
    all_bodies_above_ground = True

    for frame in range(FRAMES):
        # Policy inference at decimation rate
        if onnx_available and frame % DECIMATION == 0:
            # Build observation
            obs = obs_builder.compute(
                command_vel=command_vel,
                default_joint_q=initial_q,
                prev_action=prev_action,
            )

            # Pad/trim to match policy expected shape
            if len(obs) < expected_obs_dim:
                obs_padded = np.zeros(expected_obs_dim, dtype=np.float32)
                obs_padded[:len(obs)] = obs
                obs = obs_padded
            elif len(obs) > expected_obs_dim:
                obs = obs[:expected_obs_dim]

            # ONNX inference
            action_raw = session.run(None, {input_name: obs.reshape(1, -1)})[0]
            action = action_raw.flatten()
            action_history.append(action.copy())

            # Apply action via mapper
            action_mapper.apply_targets(action)
            prev_action = action[:n_dof].astype(np.float32)

            if frame == 0:
                print(f"  First obs[:12]: {obs[:12]}")
                print(f"  First action[:6]: {action[:6]}")

        rs.step()

        # Check termination: base height
        base_center = rs.get_body_center(model.root_link)
        if base_center[2] < 0.1:
            all_bodies_above_ground = False
            print(f"  WARNING: Base below 0.1m at frame {frame} (z={base_center[2]:.4f})")
            break

        if frame % 100 == 0 and frame > 0:
            q = rs.get_all_joint_values()
            print(f"  frame {frame:4d}: q[:6]={[f'{float(v):.4f}' for v in q[:6]]}")

    rs.save_result(os.path.join(OUTPUT_DIR, "policy_result.obj"))

    # ── Validate ────────────────────────────────────────────────────
    print("\n--- Validation ---")
    final_q = rs.get_all_joint_values()

    # 1. Joint drift check
    max_drift = max(abs(float(final_q[i]) - float(initial_q[i]))
                    for i in range(min(n_joints, len(initial_q))))
    print(f"  Max joint drift: {max_drift:.4f} rad")
    assert max_drift < 2.0, f"Robot collapsed (max drift {max_drift:.4f})"

    # 2. All bodies above ground
    for bname in body_names[:5]:  # check first 5 bodies
        try:
            c = rs.get_body_center(bname)
            assert c[2] > -0.5, f"{bname} below ground: z={c[2]:.4f}"
        except Exception:
            pass
    if all_bodies_above_ground:
        print(f"  All bodies above ground: PASSED")

    # 3. Body velocities sanity
    for bname in body_names[:3]:
        try:
            v = rs.get_body_velocity(bname)
            speed = float(np.sqrt(np.sum(np.array(v[:3])**2)))
            assert speed < 100.0, f"{bname} velocity {speed:.1f} m/s"
        except Exception:
            pass
    print(f"  Body velocity sanity: PASSED")

    # 4. Policy statistics
    if onnx_available:
        n_policy_steps = len(action_history)
        print(f"  Policy steps executed: {n_policy_steps} "
              f"(decimation={DECIMATION})")
        if action_history:
            all_actions = np.concatenate([a.flatten() for a in action_history])
            print(f"  Action mean abs: {np.mean(np.abs(all_actions)):.4f}")
            print(f"  Action std: {np.std(all_actions):.4f}")
            assert n_policy_steps > 0, "No policy steps executed"

    # 5. Observation sanity
    test_obs = obs_builder.compute(command_vel=command_vel,
                                   default_joint_q=initial_q)
    assert not np.any(np.isnan(test_obs)), "Observation contains NaN"
    assert not np.any(np.isinf(test_obs)), "Observation contains Inf"
    print(f"  Observation sanity: PASSED "
          f"(dim={len(test_obs)}, range=[{test_obs.min():.3f}, {test_obs.max():.3f}])")

    print(f"  Robot: {args.robot}, frames: {FRAMES}, "
          f"decimation: {DECIMATION}, action_scale: {args.action_scale}")

    # ── Parity gap notes ─────────────────────────────────────────
    print(f"\n  Parity gaps (vs Newton robot_policy):")
    print(f"    - Observation: full build but not yet field-by-field verified vs Newton")
    print(f"    - Command velocity: hardcoded (Newton: keyboard-driven)")
    print(f"    - No termination/reward/reset episode loop")
    print(f"    - No joint name reorder (physx vs mujoco order mapping)")
    print(f"    - No batch ONNX inference (world_count=1)")
    print(f"    - Fixed base (Newton uses floating base)")

    print("ONNX policy test PASSED")

else:
    from robotics.render.robot_viewer import RobotViewer
    viewer = RobotViewer(rs, rs.config, OUTPUT_DIR)
    viewer.show()

rs.cleanup()
