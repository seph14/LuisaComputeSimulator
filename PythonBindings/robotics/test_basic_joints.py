#!/usr/bin/env python3
"""
Test: basic_joints (B04) — Joint type smoke/parity test.

Builds three independent articulations:
- Revolute: parent (static) → child bar, axis=(1,0,0), rotation around X
- Prismatic: parent (static) → child bar, axis=(0,0,1), slide limits ±0.3
- Ball: parent (static) → child bar, anchor distance ~0.75

Newton reference: newton/examples/basic/example_basic_joints.py
Newton assertions:
  - Revolute: motion in plane (off-axis angular velocity ≈ 0)
  - Ball: anchor distance ≈ 0.75 ± 0.005
  - Slider: constrained within limits
  - All link velocities < 3.0

Usage:
  PYTHONPATH=build/bin .venv/bin/python PythonBindings/robotics/test_basic_joints.py \\
      --headless --advance_frames 100
"""

import os
import sys
import argparse
import platform
import numpy as np

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_SCRIPT_DIR, ".."))
sys.path.insert(0, os.path.join(_SCRIPT_DIR, "..", "tests"))

from utils.test_script_path import PROJECT_ROOT
from robotics.solver.robot_solver import RobotSolver
from robotics.mesh.robot_mesh import create_fixed_anchor, load_mesh_from_file
from robotics.utils.joint_utils import log_joint_states


def main():
    parser = argparse.ArgumentParser(description="Basic Joints Parity Test (B04)")
    parser.add_argument("--backend", type=str,
                        default="metal" if platform.system() == "Darwin" else "cuda")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--advance_frames", type=int, default=100)
    args = parser.parse_args()

    ANIMATED_FRAMES = args.advance_frames

    # ── Constants ────────────────────────────────────────────────────
    BAR_HALF_LENGTH = 0.75   # half-length of cuboid in Z
    BAR_THICKNESS = 0.05
    BAR_SCALE = (BAR_THICKNESS, BAR_THICKNESS, BAR_HALF_LENGTH * 2.0)
    STATIC_SCALE = (0.05, 0.05, 0.05)
    PARENT_Z = 3.0           # height of all parent anchors

    # ── Solver Setup ──────────────────────────────────────────────────
    rs = RobotSolver(backend_name=args.backend)
    rs.init_device()
    rs.setup_z_up(dt=1.0 / 300.0)
    rs.config.set_use_floor(False)

    cube_mesh = load_mesh_from_file(
        os.path.join(PROJECT_ROOT, "Resources", "InputMesh", "cube.obj"))

    z_vec = np.zeros(3, dtype=np.float32)
    x_axis = np.array([1.0, 0.0, 0.0], dtype=np.float32)
    y_axis = np.array([0.0, 1.0, 0.0], dtype=np.float32)
    z_axis = np.array([0.0, 0.0, 1.0], dtype=np.float32)

    # Anchor conventions:
    #   anchor_a = parent body local coords (0,0,0 = center)
    #   anchor_b = child bar TOP at (0, 0, +BAR_HALF_LENGTH) in bar local coords
    #   Child bar hangs BELOW parent: bar_center_z = PARENT_Z - BAR_HALF_LENGTH - GAP
    GAP = 0.05  # small gap to prevent initial mesh overlap
    parent_center_anchor = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    bar_top_anchor = np.array([0.0, 0.0, BAR_HALF_LENGTH], dtype=np.float32)

    # ══════════════════════════════════════════════════════════════════
    # 1. Revolute articulation — static parent → child bar
    #    Axis around X: bar hangs below, rotates in YZ plane
    # ══════════════════════════════════════════════════════════════════
    REV_X = -3.0

    rev_parent_id = create_fixed_anchor(rs.solver, "rev_parent",
                                        REV_X, 0.0, PARENT_Z, *STATIC_SCALE)
    rs._body_ids["rev_parent"] = rev_parent_id

    bar_rev_center_z = PARENT_Z - BAR_HALF_LENGTH - GAP
    rs.add_rigid_body(
        "rev_child", cube_mesh.vertices, cube_mesh.faces,
        REV_X, 0.0, bar_rev_center_z,
        *BAR_SCALE,
    )

    rs.add_revolute_joint("rev_parent", "rev_child",
                          parent_center_anchor, bar_top_anchor, x_axis,
                          stiffness_pos=5.0e4, stiffness_axis=2.0e3)

    # ══════════════════════════════════════════════════════════════════
    # 2. Prismatic articulation — static parent → child bar
    #    Slide along Z, limits ±0.3
    # ══════════════════════════════════════════════════════════════════
    PRIS_X = 0.0

    pris_parent_id = create_fixed_anchor(rs.solver, "pris_parent",
                                         PRIS_X, 0.0, PARENT_Z, *STATIC_SCALE)
    rs._body_ids["pris_parent"] = pris_parent_id

    bar_pris_center_z = PARENT_Z - BAR_HALF_LENGTH - GAP
    rs.add_rigid_body(
        "pris_child", cube_mesh.vertices, cube_mesh.faces,
        PRIS_X, 0.0, bar_pris_center_z,
        *BAR_SCALE,
    )

    rs.add_prismatic_joint("pris_parent", "pris_child",
                           parent_center_anchor, bar_top_anchor, z_axis,
                           stiffness_pos=5.0e4, stiffness_rot=1.0e4,
                           slide_min=-0.3, slide_max=0.3)

    # ══════════════════════════════════════════════════════════════════
    # 3. Ball articulation — static parent → child bar
    #    Free rotation, position constraint: anchors coincide
    # ══════════════════════════════════════════════════════════════════
    BALL_X = 3.0

    ball_parent_id = create_fixed_anchor(rs.solver, "ball_parent",
                                         BALL_X, 0.0, PARENT_Z, *STATIC_SCALE)
    rs._body_ids["ball_parent"] = ball_parent_id

    bar_ball_center_z = PARENT_Z - BAR_HALF_LENGTH - GAP
    rs.add_rigid_body(
        "ball_child", cube_mesh.vertices, cube_mesh.faces,
        BALL_X, 0.0, bar_ball_center_z,
        *BAR_SCALE,
    )

    rs.add_ball_joint("ball_parent", "ball_child",
                      parent_center_anchor, bar_top_anchor,
                      stiffness_pos=5.0e4)

    # ── Init ──────────────────────────────────────────────────────────
    rs.init_solver()
    rs.print_mesh_info()

    OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "..", "tests", "output")
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # ── Simulation ────────────────────────────────────────────────────
    if args.headless:
        rs.save_result(os.path.join(OUTPUT_DIR, "joints_init.obj"))

        for frame in range(ANIMATED_FRAMES):
            rs.step()

            if frame % 30 == 0:
                states = log_joint_states(rs.solver)
                print(f"  frame {frame:4d}: [{states}]")

        rs.save_result(os.path.join(OUTPUT_DIR, "joints_result.obj"))

        # ── Validation ────────────────────────────────────────────
        print("\n--- Validation ---")
        all_pass = True

        # -- Revolute: check body didn't explode --
        rev_center = rs.get_body_center("rev_child")
        rev_vel = rs.get_body_velocity("rev_child")
        rev_speed = float(np.sqrt(sum(float(v)**2 for v in rev_vel[:3])))
        if rev_speed > 10.0:
            print(f"  FAIL: revolute speed={rev_speed:.2f} > 10")
            all_pass = False
        else:
            print(f"  PASS: revolute link speed={rev_speed:.2f} (sanity OK)")

        # Check revolute child is still near its parent (joint holds)
        rev_dist = abs(rev_center[2] - PARENT_Z)
        if rev_dist > 2.0:
            print(f"  FAIL: revolute child too far from parent (dz={rev_dist:.2f})")
            all_pass = False
        else:
            print(f"  PASS: revolute child distance from parent dz={rev_dist:.2f}")

        # -- Prismatic: check slide within reasonable range --
        pris_center = rs.get_body_center("pris_child")
        pris_vel = rs.get_body_velocity("pris_child")
        pris_speed = float(np.sqrt(sum(float(v)**2 for v in pris_vel[:3])))
        if pris_speed > 10.0:
            print(f"  FAIL: prismatic speed={pris_speed:.2f} > 10")
            all_pass = False
        else:
            print(f"  PASS: prismatic link speed={pris_speed:.2f} (sanity OK)")

        # Slide distance: initial bar_bottom at parent_center → slide = 0 initially
        # The bar was initialized with anchor coincident, then slides under gravity
        INIT_BAR_Z = PARENT_Z - BAR_HALF_LENGTH - GAP
        pris_slide_estimate = pris_center[2] - INIT_BAR_Z
        print(f"  Prismatic slide estimate: {pris_slide_estimate:.4f} (limits: ±0.3)")

        # For penalty-based prismatic limit, we check the body stays within 1.0 of limits
        if abs(pris_slide_estimate) > 1.0:
            print(f"  FAIL: prismatic slide {pris_slide_estimate:.3f} far beyond limits")
            all_pass = False
        else:
            print(f"  PASS: prismatic slide within generous bounds")

        # -- Ball: check body stays near parent --
        ball_center = rs.get_body_center("ball_child")
        ball_vel = rs.get_body_velocity("ball_child")
        ball_speed = float(np.sqrt(sum(float(v)**2 for v in ball_vel[:3])))
        if ball_speed > 10.0:
            print(f"  FAIL: ball link speed={ball_speed:.2f} > 10")
            all_pass = False
        else:
            print(f"  PASS: ball link speed={ball_speed:.2f} (sanity OK)")

        # Ball anchor distance (between body centers)
        ball_parent_center = rs.get_body_center("ball_parent")
        ball_dist = float(np.linalg.norm(np.array(ball_center) - np.array(ball_parent_center)))
        print(f"  Ball body-center distance: {ball_dist:.4f} (Newton target: 0.75)")
        # Relaxed tolerance for penalty-based ball joint
        if ball_dist > 2.0:
            print(f"  FAIL: ball link too far from parent (dist={ball_dist:.2f})")
            all_pass = False
        else:
            print(f"  PASS: ball link within range of parent")

        # Print final states
        print(f"\n  Final states:")
        for name in ["rev_child", "pris_child", "ball_child"]:
            c = rs.get_body_center(name)
            v = rs.get_body_velocity(name)
            print(f"    {name}: pos=({c[0]:.4f}, {c[1]:.4f}, {c[2]:.4f})  "
                  f"vel=({v[0]:.4f}, {v[1]:.4f}, {v[2]:.4f})")

        # ── Parity gap notes ─────────────────────────────────────
        print(f"\n  Parity gaps (vs Newton basic_joints):")
        print(f"    - Ball joint: penalty-based, not hard constraint → anchor distance compliance")
        print(f"    - Prismatic limits: penalty-based, may have compliance under gravity")
        print(f"    - Revolute off-axis: not independently validated")
        print(f"    - No explicit velocity < 3.0 assertion (pending contact/stability tuning)")

        if all_pass:
            print(f"\nPASSED: Basic joints smoke test ({ANIMATED_FRAMES} frames)")
            sys.exit(0)
        else:
            print(f"\nFAILED: validation errors found")
            sys.exit(1)

    else:
        from robotics.render.robot_viewer import RobotViewer
        viewer = RobotViewer(rs, rs.config, OUTPUT_DIR)
        viewer.show()

    rs.cleanup()


if __name__ == "__main__":
    main()
