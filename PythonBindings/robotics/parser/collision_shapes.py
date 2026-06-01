"""
Collision shape descriptors and helpers (ROADMAP 3.1-3.5).

Defines primitive shapes, collision groups, and per-shape material
properties that map to the C++ collision pipeline.
"""

from dataclasses import dataclass, field
from typing import List, Optional
import numpy as np


@dataclass
class CollisionShapeDesc:
    """Descriptor for a single collision shape attached to a link."""
    shape_type: str = "box"  # box, sphere, capsule, cylinder, mesh, convex_hull
    size: List[float] = field(default_factory=lambda: [0.1, 0.1, 0.1])
    radius: float = 0.05
    length: float = 0.1
    mesh_path: str = ""
    mesh_scale: List[float] = field(default_factory=lambda: [1.0, 1.0, 1.0])
    # Transform relative to link frame
    origin_xyz: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    origin_rpy: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    # Material
    friction: float = 0.5       # mu (Coulomb friction coefficient)
    restitution: float = 0.0    # coefficient of restitution
    # Collision filtering
    collision_group: int = 1    # bitmask: which group this shape belongs to
    collision_mask: int = 0xFFFF  # bitmask: which groups this shape collides with


@dataclass
class CollisionGroupConfig:
    """Global collision group configuration for a robot."""
    # Default collision groups
    GROUP_TERRAIN: int = 1 << 0     # 1
    GROUP_ROBOT_LINK: int = 1 << 1  # 2
    GROUP_GRIPPER: int = 1 << 2     # 4
    GROUP_OBJECT: int = 1 << 3      # 8

    # Whether to disable self-collision within a single robot
    disable_self_collision: bool = True

    # Per-link-pair collision overrides
    # Keys: (link_a, link_b) tuples, Values: True=enable, False=disable
    link_pair_overrides: dict = field(default_factory=dict)

    def should_collide(self, group_a: int, group_b: int,
                       link_a: str = "", link_b: str = "") -> bool:
        """Check if two collision groups should interact."""
        # Check link-pair overrides first
        pair = (link_a, link_b)
        pair_rev = (link_b, link_a)
        if pair in self.link_pair_overrides:
            return self.link_pair_overrides[pair]
        if pair_rev in self.link_pair_overrides:
            return self.link_pair_overrides[pair_rev]

        # Check group masks
        return (group_a & group_b) != 0

    def disable_link_pair(self, link_a: str, link_b: str):
        """Disable collision between two specific links."""
        self.link_pair_overrides[(link_a, link_b)] = False

    def enable_link_pair(self, link_a: str, link_b: str):
        """Enable collision between two specific links."""
        self.link_pair_overrides[(link_a, link_b)] = True


@dataclass
class CollisionShapeLibrary:
    """Registry of collision shape descriptions for a robot."""
    shapes: List[CollisionShapeDesc] = field(default_factory=list)
    group_config: CollisionGroupConfig = field(default_factory=CollisionGroupConfig)

    def add_box(self, half_extents=(0.05, 0.05, 0.05),
                origin=(0, 0, 0), friction=0.5, group=2) -> CollisionShapeDesc:
        s = CollisionShapeDesc(shape_type="box", size=list(half_extents),
                               friction=friction, collision_group=group)
        s.origin_xyz = np.array(origin, dtype=np.float32)
        self.shapes.append(s)
        return s

    def add_sphere(self, radius=0.05, origin=(0, 0, 0),
                   friction=0.5, group=2) -> CollisionShapeDesc:
        s = CollisionShapeDesc(shape_type="sphere", radius=radius,
                               friction=friction, collision_group=group)
        s.origin_xyz = np.array(origin, dtype=np.float32)
        self.shapes.append(s)
        return s

    def add_capsule(self, radius=0.03, length=0.1, origin=(0, 0, 0),
                    friction=0.5, group=2) -> CollisionShapeDesc:
        s = CollisionShapeDesc(shape_type="capsule", radius=radius, length=length,
                               friction=friction, collision_group=group)
        s.origin_xyz = np.array(origin, dtype=np.float32)
        self.shapes.append(s)
        return s

    def add_cylinder(self, radius=0.03, length=0.1, origin=(0, 0, 0),
                     friction=0.5, group=2) -> CollisionShapeDesc:
        s = CollisionShapeDesc(shape_type="cylinder", radius=radius, length=length,
                               friction=friction, collision_group=group)
        s.origin_xyz = np.array(origin, dtype=np.float32)
        self.shapes.append(s)
        return s

    def add_mesh(self, path, scale=(1, 1, 1), origin=(0, 0, 0),
                 friction=0.5, group=2) -> CollisionShapeDesc:
        s = CollisionShapeDesc(shape_type="mesh", mesh_path=path,
                               mesh_scale=list(scale), friction=friction,
                               collision_group=group)
        s.origin_xyz = np.array(origin, dtype=np.float32)
        self.shapes.append(s)
        return s
