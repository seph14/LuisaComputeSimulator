"""
Minimal gravity test: single floating rigid body + disable_floor + no joints.
Expected: body should free-fall under gravity.
"""

import os, sys, argparse, platform
import numpy as np

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_SCRIPT_DIR, ".."))

import lcs_py as lcs
import trimesh

parser = argparse.ArgumentParser()
parser.add_argument("--backend", type=str,
                    default="metal" if platform.system() == "Darwin" else "cuda")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--advance_frames", type=int, default=60)
parser.add_argument("--fixed", action="store_true", help="Fix the body (no gravity expected)")
args = parser.parse_args()

FRAMES = args.advance_frames

# Create a simple box mesh
box = trimesh.creation.box(extents=[0.2, 0.2, 0.2])
verts = np.asarray(box.vertices, dtype=np.float64)
faces = np.asarray(box.faces, dtype=np.int32)

# Setup solver
solver = lcs.NewtonSolver()
solver.init_device(backend_name=args.backend)

config = solver.get_config()
config.set_up_axis(lcs.UpAxis.Z_UP)
config.set_implicit_dt(1.0 / 300.0)
config.set_num_substep(3)
config.set_nonlinear_iter_count(5)
config.set_pcg_iter_count(200)
config.set_use_floor(False)
config.set_use_gpu(False)
config.set_use_self_collision(False)

g = config.get_gravity()
print(f"Gravity: ({float(g.x):.3f}, {float(g.y):.3f}, {float(g.z):.3f})")

# Create and register a single rigid body
body = solver.create_world_data_from_array("box", verts, faces)
body.set_simulation_type(lcs.MaterialType.Rigid)
# Set explicit mass = 1.0 kg
body.set_physics_material_rigid(
    "StableNeoHookean",  # model (correct case!)
    0.001,                # thickness
    1e8,                  # stiffness
    0.0,                  # density (0 = use explicit mass)
    1.0,                  # mass = 1.0 kg
    0.001,                # d_hat
    0.0,                  # contact_offset
)
# Start at z=2.0, so we can observe falling
body.set_translation(0.0, 0.0, 2.0)
body.set_rotation(0.0, 0.0, 0.0)
body.set_scale_xyz(1.0, 1.0, 1.0)

if args.fixed:
    body.add_fixed_point_by_indices(np.array([0], dtype=np.int32))
    print("Mode: FIXED (body pinned, should NOT fall)")
else:
    print("Mode: FLOATING (body should free-fall under gravity)")

rid = solver.register_world_data(body)
print(f"Registered body with ID: {rid}")

solver.init_solver()

# Track position
initial_translation = np.array(solver.get_rigid_body_translation(rid), dtype=np.float64)
print(f"Initial position: ({initial_translation[0]:.4f}, {initial_translation[1]:.4f}, {initial_translation[2]:.4f})")

for frame in range(FRAMES):
    solver.physics_step_cpu()
    if frame % 15 == 0 or frame == FRAMES - 1:
        t = np.array(solver.get_rigid_body_translation(rid), dtype=np.float64)
        v = np.array(solver.get_rigid_body_velocity(rid), dtype=np.float64)
        dz = float(t[2] - initial_translation[2])
        speed = float(np.linalg.norm(v[:3]))
        print(f"  frame {frame:4d}: z={t[2]:.4f} (dz={dz:+.4f}), "
              f"vz={v[2]:.4f}, speed={speed:.4f}")

solver.cleanup()

# Expected for 60 frames at dt=1/300:
#   t = 60/300 = 0.2s, d = 0.5 * 9.8 * 0.2^2 = 0.196 m
#   v = 9.8 * 0.2 = 1.96 m/s
print(f"\nExpected after {FRAMES} frames (t={FRAMES/300:.3f}s):")
print(f"  Free fall: dz ≈ -0.5 * 9.8 * ({FRAMES/300})² = {-0.5*9.8*(FRAMES/300)**2:.3f} m")
print(f"  Velocity:  vz ≈ -9.8 * {FRAMES/300} = {-9.8*FRAMES/300:.3f} m/s")
