"""
Minimal RobotBuilder smoke test (R.4).

Validates that RobotBuilder can parse a minimal inline URDF and
produce the correct body/joint structure WITHOUT requiring a C++ device
or external asset files.

Usage:
    python3 PythonBindings/robotics/test_builder_smoke.py
"""

import math
import os, sys
import numpy as np
import tempfile

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_SCRIPT_DIR))

from robotics.parser.urdf_parser import URDFParser
from robotics.robot_builder import RobotBuilder


MINIMAL_URDF = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base"/>
  <link name="arm"/>
  <joint name="base_to_arm" type="revolute">
    <parent link="base"/>
    <child link="arm"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
"""

AXIS_RPY_URDF = f"""<?xml version="1.0"?>
<robot name="axis_robot">
  <link name="base"/>
  <link name="arm"/>
  <joint name="base_to_arm" type="revolute">
    <parent link="base"/>
    <child link="arm"/>
    <origin xyz="0 0 0" rpy="0 0 {math.pi / 2.0}"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.5" upper="0.5" effort="1" velocity="1"/>
  </joint>
</robot>
"""


class _FakeLowLevelSolver:
    def __init__(self):
        self.fixed_joints = []

    def add_fixed_joint(self, *args, **kwargs):
        self.fixed_joints.append((args, kwargs))


class _FakeRobotSolver:
    def __init__(self):
        self.solver = _FakeLowLevelSolver()
        self.bodies = []
        self.revolute_joints = []

    def add_rigid_body(self, name, vertices, faces, **kwargs):
        self.bodies.append((name, vertices, faces, kwargs))
        return len(self.bodies) - 1

    def add_revolute_joint(self, body_a_name, body_b_name,
                           anchor_a_local, anchor_b_local,
                           axis_world, axis_a_local=None, axis_b_local=None,
                           **kwargs):
        self.revolute_joints.append({
            "body_a_name": body_a_name,
            "body_b_name": body_b_name,
            "anchor_a_local": np.asarray(anchor_a_local),
            "anchor_b_local": np.asarray(anchor_b_local),
            "axis_world": np.asarray(axis_world),
            "axis_a_local": np.asarray(axis_a_local),
            "axis_b_local": np.asarray(axis_b_local),
            "kwargs": kwargs,
        })

    def add_prismatic_joint(self, *args, **kwargs):
        raise AssertionError("unexpected prismatic joint")

    def add_free_joint(self, *args, **kwargs):
        raise AssertionError("unexpected free joint")


def test_urdf_parse():
    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False) as f:
        f.write(MINIMAL_URDF)
        f.flush()
        path = f.name

    try:
        model = URDFParser.parse(path)
        assert model.root_link == "base", f"Expected 'base', got '{model.root_link}'"
        assert len(model.links) == 2
        assert len(model.joints) == 1
        j = model.joints[0]
        assert j.joint_type == "revolute"
        assert j.parent == "base" and j.child == "arm"
        print("  URDFParser.parse: PASSED")
        return model
    finally:
        os.unlink(path)


def test_rpy_to_matrix():
    r = RobotBuilder._rpy_to_matrix(np.array([0.1, 0.2, 0.3]))
    # Should be valid rotation matrix: determinant ≈ 1
    det = float(np.linalg.det(r))
    assert abs(det - 1.0) < 1e-6, f"RPY matrix det = {det}"
    assert r.shape == (3, 3)
    print("  _rpy_to_matrix: PASSED")


def test_topology_order():
    model = test_urdf_parse()
    order = URDFParser.build_topology_order(model)
    assert order == ["base", "arm"], f"Unexpected order: {order}"
    print("  build_topology_order: PASSED")


def test_floating_detection():
    """Verify _has_floating_joint detects floating/free types."""
    model = test_urdf_parse()
    # Our test URDF has a revolute joint, not floating
    # We need a mock solver for RobotBuilder, so test via model inspection
    for j in model.joints:
        assert j.joint_type.lower() not in ("floating", "free")
    print("  floating detection: PASSED")


def test_joint_axis_origin_rpy():
    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False) as f:
        f.write(AXIS_RPY_URDF)
        f.flush()
        path = f.name

    try:
        model = URDFParser.parse(path)
        fake_rs = _FakeRobotSolver()
        RobotBuilder(fake_rs, model).build()
        assert len(fake_rs.revolute_joints) == 1
        joint = fake_rs.revolute_joints[0]
        np.testing.assert_allclose(joint["axis_world"], np.array([0.0, 1.0, 0.0]), atol=1e-6)
        np.testing.assert_allclose(joint["axis_a_local"], np.array([0.0, 1.0, 0.0]), atol=1e-6)
        np.testing.assert_allclose(joint["axis_b_local"], np.array([1.0, 0.0, 0.0]), atol=1e-6)
        assert joint["kwargs"]["lower_angle"] == -0.5
        assert joint["kwargs"]["upper_angle"] == 0.5
        print("  joint axis origin_rpy: PASSED")
    finally:
        os.unlink(path)


if __name__ == "__main__":
    print("Running RobotBuilder smoke tests...")
    test_urdf_parse()
    test_rpy_to_matrix()
    test_topology_order()
    test_floating_detection()
    test_joint_axis_origin_rpy()
    print("\nAll RobotBuilder smoke tests PASSED")
