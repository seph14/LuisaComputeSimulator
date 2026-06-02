"""
World metadata structures for multi-world body/joint tracking (ROADMAP P2.1).

Provides BodyMeta, JointMeta, and WorldMeta dataclasses that track
the mapping from (world_id, body_name) to C++ registration_id and
from (world_id, joint_info) to C++ joint_index.

Used by RobotSolver.replicate() and ArticulationView.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional
import numpy as np


@dataclass
class BodyMeta:
    """Metadata for one body instance in one world."""
    name: str               # original body name (e.g. "cart")
    world_id: int           # world index (0-based)
    registration_id: int    # C++ registration_id
    fixed: bool = False     # whether this body is a fixed base


@dataclass
class JointMeta:
    """Metadata for one joint instance in one world."""
    name: str               # original joint name (e.g. "cart_pole1_revolute")
    parent_name: str        # original parent body name
    child_name: str         # original child body name
    joint_type: str         # "fixed" | "prismatic" | "revolute" | "ball" | "free"
    world_id: int           # world index (0-based)
    joint_index: int        # C++ joint index (flat, global)
    dof_index: int = -1     # 0-based DOF index within this world (-1 for fixed/free)


@dataclass
class WorldMeta:
    """
    Aggregate metadata for all bodies and joints across all worlds.

    Body naming convention: world_{w}/{original_body_name}
    Joint naming convention: world_{w}/{parent}_{child}_{type}
    """
    world_count: int = 0
    body_count_per_world: int = 0
    joint_count_per_world: int = 0
    dof_per_world: int = 0

    # world_id -> {body_name -> BodyMeta}
    body_map: Dict[int, Dict[str, BodyMeta]] = field(default_factory=dict)

    # world_id -> {joint_name -> JointMeta}
    joint_map: Dict[int, Dict[str, JointMeta]] = field(default_factory=dict)

    # registration_id -> BodyMeta (reverse lookup)
    rid_to_body: Dict[int, BodyMeta] = field(default_factory=dict)

    # joint_index -> JointMeta (reverse lookup)
    jidx_to_joint: Dict[int, JointMeta] = field(default_factory=dict)

    # Initial state snapshots for reset (P2.4)
    # {registration_id: (tx,ty,tz, qx,qy,qz,qw)}
    initial_body_poses: Dict[int, np.ndarray] = field(default_factory=dict)
    # {joint_index: value}
    initial_joint_q: Dict[int, float] = field(default_factory=dict)

    def get_body_meta(self, name: str, world_id: int = 0) -> Optional[BodyMeta]:
        """Get BodyMeta by body name and world."""
        world_bodies = self.body_map.get(world_id, {})
        return world_bodies.get(name)

    def get_body_rid(self, name: str, world_id: int = 0) -> int:
        """Get registration_id by body name and world. Returns -1 if not found."""
        bm = self.get_body_meta(name, world_id)
        return bm.registration_id if bm else -1

    def get_joint_meta(self, parent_name: str, child_name: str,
                       world_id: int = 0) -> Optional[JointMeta]:
        """Get JointMeta by parent/child names and world."""
        world_joints = self.joint_map.get(world_id, {})
        for jname, jm in world_joints.items():
            if jm.parent_name == parent_name and jm.child_name == child_name:
                return jm
        return None

    def get_joint_index(self, parent_name: str, child_name: str,
                        world_id: int = 0) -> int:
        """Get joint index by parent/child names and world. Returns -1 if not found."""
        jm = self.get_joint_meta(parent_name, child_name, world_id)
        return jm.joint_index if jm else -1

    def world_body_names(self, world_id: int = 0) -> List[str]:
        """List body names in a given world."""
        world_bodies = self.body_map.get(world_id, {})
        return list(world_bodies.keys())

    def world_joint_indices(self, world_id: int = 0) -> List[int]:
        """List joint indices in a given world, in registration order.

        NOTE: This sorts by joint_index which assumes joints were registered
        sequentially per world (as replicate() does). If joints are added
        in a different order (e.g. all fixed joints then all revolute),
        the returned order will follow the C++ joint type concatenation
        (fixed → prismatic → revolute → ball → free), not per-world order."""
        world_joints = self.joint_map.get(world_id, {})
        # Sort by joint_index to get registration order
        sorted_joints = sorted(world_joints.values(), key=lambda jm: jm.joint_index)
        return [jm.joint_index for jm in sorted_joints]

    def world_dof_indices(self, world_id: int = 0) -> List[int]:
        """List DOF joint indices (excludes fixed/free) in registration order."""
        world_joints = self.joint_map.get(world_id, {})
        dof_joints = [jm for jm in world_joints.values() if jm.dof_index >= 0]
        dof_joints.sort(key=lambda jm: jm.dof_index)
        return [jm.joint_index for jm in dof_joints]
