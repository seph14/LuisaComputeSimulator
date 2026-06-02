"""
Action mapper for RL policy pipelines (ROADMAP P5.2).

Maps policy outputs to joint target positions:
  - action_scale: scale the raw policy output
  - default_pose_offset: offset from default joint positions
  - joint name reorder: map policy output indices to joint indices

Usage:
    mapper = ActionMapper(rs, dof_joint_indices, default_joint_q,
                          action_scale=0.5)
    targets = mapper.compute_targets(action_vector)
"""

import numpy as np


class ActionMapper:
    """
    Map ONNX policy outputs to joint target positions.

    Supports:
      - action_scale: multiplier applied to raw action values
      - default pose offset: add action to default joint positions
      - joint name reorder: map policy output order to solver joint order
        (e.g. lab_to_mujoco or physx order mapping)
    """

    def __init__(self, robot_solver, dof_joint_indices: list,
                 default_joint_q: np.ndarray = None,
                 action_scale: float = 0.5,
                 policy_to_dof_map: dict = None):
        """
        Args:
            robot_solver: RobotSolver instance.
            dof_joint_indices: Global joint indices in DOF order.
            default_joint_q: [num_dof] default/rest joint positions.
            action_scale: Multiplier applied to policy actions.
            policy_to_dof_map: If not None, maps policy output index →
                               DOF index (for joint name reorder).
                               Example: {0: 2, 1: 0, 2: 1} means
                               policy[0] → DOF[2], policy[1] → DOF[0].
        """
        self._rs = robot_solver
        self._dof_indices = list(dof_joint_indices)
        self._num_dof = len(dof_joint_indices)

        if default_joint_q is None:
            self._default_q = np.zeros(self._num_dof, dtype=np.float64)
        else:
            self._default_q = np.asarray(default_joint_q, dtype=np.float64)

        self._action_scale = float(action_scale)

        # Build reorder map
        if policy_to_dof_map is None:
            self._reorder = list(range(self._num_dof))
        else:
            self._reorder = [policy_to_dof_map.get(i, i)
                             for i in range(self._num_dof)]

    @property
    def action_scale(self) -> float:
        return self._action_scale

    @action_scale.setter
    def action_scale(self, value: float):
        self._action_scale = float(value)

    @property
    def default_q(self) -> np.ndarray:
        return self._default_q.copy()

    @default_q.setter
    def default_q(self, value: np.ndarray):
        self._default_q = np.asarray(value, dtype=np.float64)

    def compute_targets(self, action: np.ndarray) -> np.ndarray:
        """
        Compute joint target positions from policy action.

        target = default_q + action * action_scale

        Args:
            action: [num_actions] raw policy output.

        Returns:
            targets: [num_dof] joint target positions (float64).
        """
        action = np.asarray(action, dtype=np.float64).flatten()
        targets = self._default_q.copy()

        for i in range(min(len(action), len(self._reorder))):
            dof_idx = self._reorder[i]
            if dof_idx < self._num_dof:
                targets[dof_idx] += float(action[i]) * self._action_scale

        return targets

    def apply_targets(self, action: np.ndarray, world_id: int = 0):
        """
        Compute targets and apply them to the solver immediately.

        Args:
            action: [num_actions] raw policy output.
            world_id: World index for multi-world scenarios.
        """
        targets = self.compute_targets(action)
        world_offset = world_id * self._num_dof  # TODO: use WorldMeta offset
        for d in range(self._num_dof):
            jidx = self._dof_indices[d] + world_offset
            self._rs.set_joint_target_pos(jidx, float(targets[d]))
