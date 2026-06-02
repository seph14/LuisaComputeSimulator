"""
Observation builder for RL policy pipelines (ROADMAP P5.1).

Computes policy observations from solver state:
  - Base linear velocity in body frame
  - Base angular velocity in body frame
  - Projected gravity in body frame
  - Command velocity (user input or default forward)
  - Joint position error (q - default_q)
  - Joint velocities
  - Previous action

Typical shape: [12 + 3 * num_dof] for Newton/MuJoCo-style policies.

Usage:
    builder = ObservationBuilder(rs, body_names, dof_joint_indices)
    obs = builder.compute(command_vel, default_joint_q, prev_action)
"""

import numpy as np


class ObservationBuilder:
    """
    Build policy observation vectors from RobotSolver state.

    Observation layout (Newton/MuJoCo convention):
        [0:3]   base linear velocity (body frame)
        [3:6]   base angular velocity (body frame)
        [6:9]   projected gravity (body frame)
        [9:12]  command velocity
        [12:12+dof]         joint position error (q - default_q)
        [12+dof:12+2*dof]   joint velocity
        [12+2*dof:12+3*dof] previous action
    """

    def __init__(self, robot_solver, body_names: list,
                 dof_joint_indices: list,
                 base_body_name: str = None,
                 gravity: np.ndarray = None):
        """
        Args:
            robot_solver: RobotSolver instance (after init_solver).
            body_names: List of body names for body-frame queries.
            dof_joint_indices: Joint indices (flat, global) in DOF order.
            base_body_name: Name of the base/root body. If None, uses first body.
            gravity: World-frame gravity vector [3]. If None, reads from
                     solver config (supports Y-up, Z-up, or custom).
        """
        self._rs = robot_solver
        self._body_names = list(body_names)
        self._dof_indices = list(dof_joint_indices)
        self._num_dof = len(dof_joint_indices)
        self._base_body = base_body_name if base_body_name else body_names[0]

        # Gravity: auto-detect from solver config if not provided
        if gravity is not None:
            self._gravity = np.asarray(gravity, dtype=np.float64)
        else:
            g = robot_solver.config.get_gravity()
            self._gravity = np.array([float(g.x), float(g.y), float(g.z)],
                                     dtype=np.float64)

        # Full observation dimension: 12 + 3 * num_dof
        self._obs_dim = 12 + 3 * self._num_dof

    @property
    def obs_dim(self) -> int:
        return self._obs_dim

    @property
    def num_dof(self) -> int:
        return self._num_dof

    def compute(self, command_vel: np.ndarray = None,
                default_joint_q: np.ndarray = None,
                prev_action: np.ndarray = None,
                world_id: int = 0) -> np.ndarray:
        """
        Compute observation vector.

        Args:
            command_vel: [3] command velocity in world frame (default: zeros)
            default_joint_q: [num_dof] default/rest joint positions for error calc
            prev_action: [num_dof] previous action (default: zeros)
            world_id: World index for multi-world scenarios.

        Returns:
            obs: [12 + 3 * num_dof] observation vector (float32)
        """
        if command_vel is None:
            command_vel = np.zeros(3, dtype=np.float32)
        if default_joint_q is None:
            default_joint_q = np.zeros(self._num_dof, dtype=np.float32)
        if prev_action is None:
            prev_action = np.zeros(self._num_dof, dtype=np.float32)

        obs = np.zeros(self._obs_dim, dtype=np.float32)

        # ── 1. Base linear velocity in body frame ──────────────────
        base_vel = self._rs.get_body_velocity(self._base_body, world_id)
        base_linear_vel = np.array(base_vel[:3], dtype=np.float64)  # world frame
        _, base_quat = self._rs.get_body_pose(self._base_body, world_id)
        # quaternion wxyz → rotate vector from world to body frame
        base_linear_body = self._quat_rotate_inv(base_quat, base_linear_vel)
        obs[0:3] = base_linear_body.astype(np.float32)

        # ── 2. Base angular velocity in body frame ─────────────────
        base_ang_vel = np.array(base_vel[3:6], dtype=np.float64)  # world frame
        base_ang_body = self._quat_rotate_inv(base_quat, base_ang_vel)
        obs[3:6] = base_ang_body.astype(np.float32)

        # ── 3. Projected gravity in body frame ─────────────────────
        # Gravity direction auto-detected from solver config (supports Y-up/Z-up/custom)
        gravity_body = self._quat_rotate_inv(base_quat, self._gravity)
        obs[6:9] = gravity_body.astype(np.float32)

        # ── 4. Command velocity ────────────────────────────────────
        obs[9:12] = np.asarray(command_vel, dtype=np.float32)[:3]

        # ── 5. Joint position error ────────────────────────────────
        all_joint_q = np.array(self._rs.get_all_joint_values(), dtype=np.float64)
        for d, jidx in enumerate(self._dof_indices):
            current_q = float(all_joint_q[jidx]) if jidx < len(all_joint_q) else 0.0
            obs[12 + d] = float(current_q - default_joint_q[d])

        # ── 6. Joint velocities ────────────────────────────────────
        all_joint_qd = np.array(self._rs.get_all_joint_velocities(), dtype=np.float64)
        for d, jidx in enumerate(self._dof_indices):
            qd = float(all_joint_qd[jidx]) if jidx < len(all_joint_qd) else 0.0
            obs[12 + self._num_dof + d] = qd

        # ── 7. Previous action ─────────────────────────────────────
        obs[12 + 2 * self._num_dof:12 + 3 * self._num_dof] = np.asarray(
            prev_action, dtype=np.float32)[:self._num_dof]

        return obs

    @staticmethod
    def _quat_rotate_inv(quat_wxyz, vec):
        """
        Rotate vector from world frame to body frame using quaternion inverse.

        quat_wxyz: [qw, qx, qy, qz] — rotation quaternion (body→world).
        vec: [3] vector in world frame.

        Returns: [3] vector in body frame.
        """
        qw, qx, qy, qz = float(quat_wxyz[0]), float(quat_wxyz[1]), \
                          float(quat_wxyz[2]), float(quat_wxyz[3])
        # Conjugate quaternion: q* = [w, -x, -y, -z]
        cw, cx, cy, cz = qw, -qx, -qy, -qz
        vx, vy, vz = float(vec[0]), float(vec[1]), float(vec[2])

        # q * v * q*: first compute q * (0, v) * q*
        # Using: q * p * q* where p = (0, vx, vy, vz)
        # Result: rotate v by quaternion
        # Formula: v' = v + 2*cross(q.xyz, cross(q.xyz, v) + q.w*v)
        # For inverse: negate q.xyz

        # Optimized body-frame rotation using q_inv = conjugate(q)
        # v_body = q_inv * v_world * q
        # = v + 2q_w * (q_xyz × v) + 2(q_xyz × (q_xyz × v))
        x, y, z = -qx, -qy, -qz  # conjugate xyz
        w = qw

        # cross1 = q_xyz × v
        cx1 = y * vz - z * vy
        cy1 = z * vx - x * vz
        cz1 = x * vy - y * vx

        # cross2 = q_xyz × cross1
        cx2 = y * cz1 - z * cy1
        cy2 = z * cx1 - x * cz1
        cz2 = x * cy1 - y * cx1

        vx_out = float(vx) + 2.0 * (w * cx1 + cx2)
        vy_out = float(vy) + 2.0 * (w * cy1 + cy2)
        vz_out = float(vz) + 2.0 * (w * cz1 + cz2)

        return np.array([vx_out, vy_out, vz_out], dtype=np.float64)
