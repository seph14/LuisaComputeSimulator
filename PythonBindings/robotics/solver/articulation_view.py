"""
ArticulationView: Batched read/write interface over multiple robot instances.

Provides tensor-shaped views over N replicated worlds:
  - Joint DOFs:   [world_count, dof_per_robot]
  - Body poses:   [world_count, body_count, 7]   (tx,ty,tz, qx,qy,qz,qw)
  - Body velocity: [world_count, body_count, 6]   (vx,vy,vz, wx,wy,wz)

Used by RL training loops and multi-world parity tests (ROADMAP P2.3).
"""

import numpy as np


class ArticulationView:
    """
    Batched read/write interface over N robot instances in a single solver.

    Usage:
        rs = RobotSolver(...)
        # ... build robot, rs.replicate(..., world_count=4) ...
        rs.init_solver()
        view = ArticulationView(rs, body_names=["cart","pole1","pole2"],
                                joint_indices_w0=[0, 1])  # world_0 DOF order
        q = view.get_joint_q()        # [4, 2]
        qd = view.get_joint_qd()      # [4, 2]
        poses = view.get_body_pose()  # [4, 3, 7]
    """

    def __init__(self, robot_solver, body_names: list, joint_indices_w0: list):
        """
        Args:
            robot_solver: RobotSolver instance (after init_solver()).
            body_names: List of body names in canonical order (original names
                        without world prefix, e.g. ["cart", "pole1", "pole2"]).
            joint_indices_w0: List of joint indices for world 0, in DOF order
                              (non-fixed, non-free joints only).
        """
        self._rs = robot_solver
        self._solver = robot_solver.solver
        self._meta = robot_solver._world_meta
        self._body_names = list(body_names)
        self._joint_indices_w0 = list(joint_indices_w0)

        self._world_count = self._meta.world_count
        self._body_count = len(body_names)
        self._dof = len(joint_indices_w0)

        if self._world_count == 0:
            # Single-world mode: treat as world_count=1
            self._world_count = 1
            # Build lookup from body_ids
            self._body_lut = np.zeros((1, self._body_count), dtype=np.int32)
            for b, bname in enumerate(body_names):
                self._body_lut[0, b] = robot_solver._body_ids.get(bname, -1)
            # Build DOF lookup from _joint_index_map
            self._dof_lut = np.zeros((1, self._dof), dtype=np.int32)
            for d, jidx in enumerate(joint_indices_w0):
                self._dof_lut[0, d] = jidx
        else:
            self._body_lut = self._build_body_lut()
            self._dof_lut = self._build_dof_lut()

    def _build_body_lut(self):
        """Build [world_count, body_count] lookup: world w, body b -> registration_id."""
        lut = np.zeros((self._world_count, self._body_count), dtype=np.int32)
        for w in range(self._world_count):
            for b, bname in enumerate(self._body_names):
                lut[w, b] = self._meta.get_body_rid(bname, w)
        return lut

    def _build_dof_lut(self):
        """Build [world_count, dof] lookup: world w, dof d -> global joint_index.

        Uses WorldMeta.world_dof_indices() for correct per-world DOF mapping,
        which handles non-uniform joint registration orders (e.g. manual multi-robot scenes)."""
        lut = np.zeros((self._world_count, self._dof), dtype=np.int32)
        for w in range(self._world_count):
            indices = self._meta.world_dof_indices(w)
            if len(indices) != self._dof:
                # If world w has a different DOF count, fall back to offset calculation
                jpw = self._meta.joint_count_per_world
                for d in range(self._dof):
                    lut[w, d] = self._joint_indices_w0[d] + w * jpw
            else:
                for d, jidx in enumerate(indices):
                    lut[w, d] = jidx
        return lut

    # ── Joint queries ────────────────────────────────────────────

    def get_joint_q(self) -> np.ndarray:
        """Return joint positions: [world_count, dof]."""
        all_q = np.array(self._rs.get_all_joint_values(), dtype=np.float64)
        result = np.zeros((self._world_count, self._dof), dtype=np.float64)
        for w in range(self._world_count):
            for d in range(self._dof):
                jidx = int(self._dof_lut[w, d])
                if jidx < len(all_q):
                    result[w, d] = all_q[jidx]
        return result

    def get_joint_qd(self) -> np.ndarray:
        """Return joint velocities: [world_count, dof]."""
        all_qd = np.array(self._rs.get_all_joint_velocities(), dtype=np.float64)
        result = np.zeros((self._world_count, self._dof), dtype=np.float64)
        for w in range(self._world_count):
            for d in range(self._dof):
                jidx = int(self._dof_lut[w, d])
                if jidx < len(all_qd):
                    result[w, d] = all_qd[jidx]
        return result

    def set_joint_target_pos(self, targets: np.ndarray):
        """
        Set joint targets for all worlds.
        Args:
            targets: [world_count, dof] or [dof] (broadcast to all worlds).

        Performance note: O(world_count × dof) Python→C++ crossings per call.
        For RL training with world_count=100, dof=12, this is 1200 calls/frame.
        A future C++ batch set_joint_target_pos_batch() would reduce this to 1 call.
        """
        targets = np.asarray(targets, dtype=np.float64)
        if targets.ndim == 1:
            targets = np.tile(targets, (self._world_count, 1))
        for w in range(self._world_count):
            for d in range(self._dof):
                jidx = int(self._dof_lut[w, d])
                self._rs.set_joint_target_pos(jidx, float(targets[w, d]))

    # ── Body queries ─────────────────────────────────────────────

    def get_body_pose(self) -> np.ndarray:
        """Return body poses: [world_count, body_count, 7] (tx,ty,tz, qx,qy,qz,qw)."""
        result = np.zeros((self._world_count, self._body_count, 7), dtype=np.float64)
        for w in range(self._world_count):
            for b in range(self._body_count):
                rid = int(self._body_lut[w, b])
                if rid >= 0:
                    t = np.array(self._solver.get_rigid_body_translation(rid),
                                 dtype=np.float64)
                    q = np.array(self._solver.get_rigid_body_rotation_quaternion(rid),
                                 dtype=np.float64)
                    result[w, b, :3] = t
                    result[w, b, 3:] = q
        return result

    def get_body_velocity(self) -> np.ndarray:
        """Return body velocities: [world_count, body_count, 6] (vx,vy,vz, wx,wy,wz)."""
        result = np.zeros((self._world_count, self._body_count, 6), dtype=np.float64)
        for w in range(self._world_count):
            for b in range(self._body_count):
                rid = int(self._body_lut[w, b])
                if rid >= 0:
                    v = np.array(self._solver.get_rigid_body_velocity(rid),
                                 dtype=np.float64)
                    result[w, b, :] = v
        return result

    @property
    def world_count(self) -> int:
        return self._world_count

    @property
    def body_count(self) -> int:
        return self._body_count

    @property
    def dof(self) -> int:
        return self._dof

    @property
    def body_names(self) -> list:
        return list(self._body_names)
