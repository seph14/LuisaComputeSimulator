"""
URDF parser for LuisaComputeSimulator robotics pipeline (ROADMAP 2.1).

Parses URDF XML files and extracts link/joint descriptors needed for
RobotBuilder scene construction.  Uses Python stdlib xml.etree.ElementTree
(no external dependencies).
"""

import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Tuple

import numpy as np


# ── Data classes ─────────────────────────────────────────────────────

@dataclass
class URDFInertial:
    mass: float = 0.0
    com: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    inertia: np.ndarray = field(default_factory=lambda: np.zeros((3, 3), dtype=np.float32))
    # diagonal form: ixx, iyy, izz, ixy, ixz, iyz
    ixx: float = 0.0
    iyy: float = 0.0
    izz: float = 0.0
    ixy: float = 0.0
    ixz: float = 0.0
    iyz: float = 0.0


@dataclass
class URDFVisual:
    name: str = ""
    geometry_type: str = ""  # "mesh", "box", "sphere", "cylinder", "capsule"
    geometry_data: dict = field(default_factory=dict)  # e.g. {"filename": "...", "scale": [...]}
    origin_xyz: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    origin_rpy: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))


@dataclass
class URDFCollision:
    name: str = ""
    geometry_type: str = ""
    geometry_data: dict = field(default_factory=dict)
    origin_xyz: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    origin_rpy: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))


@dataclass
class URDFLink:
    name: str = ""
    inertial: Optional[URDFInertial] = None
    visuals: List[URDFVisual] = field(default_factory=list)
    collisions: List[URDFCollision] = field(default_factory=list)


@dataclass
class URDFJoint:
    name: str = ""
    joint_type: str = ""  # "revolute", "prismatic", "fixed", "continuous", "floating", "planar"
    parent: str = ""
    child: str = ""
    origin_xyz: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    origin_rpy: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    axis: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0], dtype=np.float32))
    lower_limit: float = 0.0
    upper_limit: float = 0.0
    effort_limit: float = 0.0
    velocity_limit: float = 0.0
    # Drive parameters (optional, from dynamics element)
    damping: float = 0.0
    friction: float = 0.0


@dataclass
class URDFRobotModel:
    """Parsed URDF robot model."""
    name: str = ""
    links: Dict[str, URDFLink] = field(default_factory=dict)
    joints: List[URDFJoint] = field(default_factory=list)
    # Parent-child link tree
    parent_map: Dict[str, str] = field(default_factory=dict)  # child_link -> parent_link
    child_map: Dict[str, List[str]] = field(default_factory=dict)  # parent_link -> [child_links]
    # Root link (the one with no parent)
    root_link: str = ""


# ── Parser ───────────────────────────────────────────────────────────

class URDFParser:
    """Parse URDF XML files into URDFRobotModel."""

    @staticmethod
    def _parse_xyz_rpy(origin_elem) -> Tuple[np.ndarray, np.ndarray]:
        """Parse <origin xyz="..." rpy="..."/> element."""
        xyz = np.zeros(3, dtype=np.float32)
        rpy = np.zeros(3, dtype=np.float32)
        if origin_elem is not None:
            xyz_str = origin_elem.get("xyz", "0 0 0")
            rpy_str = origin_elem.get("rpy", "0 0 0")
            xyz = np.array([float(v) for v in xyz_str.split()], dtype=np.float32)
            rpy = np.array([float(v) for v in rpy_str.split()], dtype=np.float32)
        return xyz, rpy

    @staticmethod
    def _parse_inertial(inertial_elem) -> Optional[URDFInertial]:
        """Parse <inertial> element."""
        if inertial_elem is None:
            return None
        inertial = URDFInertial()

        mass_elem = inertial_elem.find("mass")
        if mass_elem is not None:
            inertial.mass = float(mass_elem.get("value", "0"))

        com_elem = inertial_elem.find("origin")
        if com_elem is not None:
            inertial.com, _ = URDFParser._parse_xyz_rpy(com_elem)

        inertia_elem = inertial_elem.find("inertia")
        if inertia_elem is not None:
            inertial.ixx = float(inertia_elem.get("ixx", "0"))
            inertial.iyy = float(inertia_elem.get("iyy", "0"))
            inertial.izz = float(inertia_elem.get("izz", "0"))
            inertial.ixy = float(inertia_elem.get("ixy", "0"))
            inertial.ixz = float(inertia_elem.get("ixz", "0"))
            inertial.iyz = float(inertia_elem.get("iyz", "0"))
            inertial.inertia = np.array([
                [inertial.ixx, inertial.ixy, inertial.ixz],
                [inertial.ixy, inertial.iyy, inertial.iyz],
                [inertial.ixz, inertial.iyz, inertial.izz],
            ], dtype=np.float32)

        return inertial

    @staticmethod
    def _parse_geometry(geom_elem) -> Tuple[str, dict]:
        """Parse <geometry> element. Returns (type, data_dict)."""
        if geom_elem is None:
            return "", {}
        for child in geom_elem:
            tag = child.tag
            data = dict(child.attrib)
            if tag == "mesh":
                data["filename"] = data.get("filename", "")
                scale_str = data.get("scale", "1 1 1")
                data["scale"] = [float(v) for v in scale_str.split()]
                return "mesh", data
            elif tag == "box":
                size_str = data.get("size", "0 0 0")
                data["size"] = [float(v) for v in size_str.split()]
                return "box", data
            elif tag == "sphere":
                data["radius"] = float(data.get("radius", "0"))
                return "sphere", data
            elif tag == "cylinder":
                data["radius"] = float(data.get("radius", "0"))
                data["length"] = float(data.get("length", "0"))
                return "cylinder", data
            elif tag == "capsule":
                data["radius"] = float(data.get("radius", "0"))
                data["length"] = float(data.get("length", "0"))
                return "capsule", data
        return "", {}

    @staticmethod
    def _parse_visual(visual_elem) -> URDFVisual:
        """Parse <visual> element."""
        visual = URDFVisual(name=visual_elem.get("name", ""))
        visual.origin_xyz, visual.origin_rpy = URDFParser._parse_xyz_rpy(
            visual_elem.find("origin"))
        geom = visual_elem.find("geometry")
        visual.geometry_type, visual.geometry_data = URDFParser._parse_geometry(geom)
        return visual

    @staticmethod
    def _parse_collision(collision_elem) -> URDFCollision:
        """Parse <collision> element."""
        collision = URDFCollision(name=collision_elem.get("name", ""))
        collision.origin_xyz, collision.origin_rpy = URDFParser._parse_xyz_rpy(
            collision_elem.find("origin"))
        geom = collision_elem.find("geometry")
        collision.geometry_type, collision.geometry_data = URDFParser._parse_geometry(geom)
        return collision

    @staticmethod
    def parse(urdf_path: str) -> URDFRobotModel:
        """Parse a URDF file and return a URDFRobotModel."""
        tree = ET.parse(urdf_path)
        root = tree.getroot()

        model = URDFRobotModel()
        model.name = root.get("name", os.path.splitext(os.path.basename(urdf_path))[0])

        # Parse links
        for link_elem in root.findall("link"):
            link = URDFLink(name=link_elem.get("name", ""))

            link.inertial = URDFParser._parse_inertial(link_elem.find("inertial"))

            for vis_elem in link_elem.findall("visual"):
                link.visuals.append(URDFParser._parse_visual(vis_elem))

            for col_elem in link_elem.findall("collision"):
                link.collisions.append(URDFParser._parse_collision(col_elem))

            model.links[link.name] = link

        # Parse joints
        for joint_elem in root.findall("joint"):
            joint = URDFJoint(
                name=joint_elem.get("name", ""),
                joint_type=joint_elem.get("type", "fixed"),
            )

            # Parent/child links
            parent_elem = joint_elem.find("parent")
            child_elem = joint_elem.find("child")
            if parent_elem is not None:
                joint.parent = parent_elem.get("link", "")
            if child_elem is not None:
                joint.child = child_elem.get("link", "")

            # Origin
            joint.origin_xyz, joint.origin_rpy = URDFParser._parse_xyz_rpy(
                joint_elem.find("origin"))

            # Axis
            axis_elem = joint_elem.find("axis")
            if axis_elem is not None:
                axis_str = axis_elem.get("xyz", "1 0 0")
                joint.axis = np.array([float(v) for v in axis_str.split()], dtype=np.float32)

            # Limits
            limit_elem = joint_elem.find("limit")
            if limit_elem is not None:
                joint.lower_limit = float(limit_elem.get("lower", "0"))
                joint.upper_limit = float(limit_elem.get("upper", "0"))
                joint.effort_limit = float(limit_elem.get("effort", "0"))
                joint.velocity_limit = float(limit_elem.get("velocity", "0"))

            # Dynamics (drive parameters)
            dynamics_elem = joint_elem.find("dynamics")
            if dynamics_elem is not None:
                joint.damping = float(dynamics_elem.get("damping", "0"))
                joint.friction = float(dynamics_elem.get("friction", "0"))

            # Track parent-child relationships
            model.parent_map[joint.child] = joint.parent
            if joint.parent not in model.child_map:
                model.child_map[joint.parent] = []
            model.child_map[joint.parent].append(joint.child)

            model.joints.append(joint)

        # Find root link (the one with no parent)
        all_links = set(model.links.keys())
        children = set(model.parent_map.keys())
        roots = all_links - children
        model.root_link = roots.pop() if roots else (list(all_links)[0] if all_links else "")

        return model

    @staticmethod
    def get_chain_joints(model: URDFRobotModel, start_link: str, end_link: str) -> List[URDFJoint]:
        """Get ordered list of joints from start_link to end_link."""
        # Build path from end_link up to start_link
        path = []
        current = end_link
        while current != start_link and current in model.parent_map:
            parent = model.parent_map[current]
            for j in model.joints:
                if j.parent == parent and j.child == current:
                    path.insert(0, j)
                    break
            current = parent
        return path

    @staticmethod
    def build_topology_order(model: URDFRobotModel) -> List[str]:
        """Return links in topological order (root first, BFS)."""
        order = []
        queue = [model.root_link] if model.root_link else []
        while queue:
            link = queue.pop(0)
            order.append(link)
            for child in model.child_map.get(link, []):
                queue.append(child)
        return order
