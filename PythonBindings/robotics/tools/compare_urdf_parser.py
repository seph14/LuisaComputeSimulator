"""
Compare the local URDFParser against an independent URDF reference parser.

The tool prefers mature third-party parsers when installed:
  1. yourdfpy
  2. urdfpy

If neither package is available, it falls back to a small XML reference parser
implemented in this file. The fallback intentionally does not reuse
robotics.parser.urdf_parser so it can still catch local parser regressions in
counts, names, topology, origins, axes, limits, inertial data, and geometry.

Example:
    PYTHONPATH=PythonBindings .venv/bin/python \
        PythonBindings/robotics/tools/compare_urdf_parser.py \
        PythonBindings/robotics/assets/unitree_h1/urdf/h1.urdf
"""

from __future__ import annotations

import argparse
import importlib.util
import math
import os
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Any

import numpy as np


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PYTHON_BINDINGS_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
if PYTHON_BINDINGS_DIR not in sys.path:
    sys.path.insert(0, PYTHON_BINDINGS_DIR)

from robotics.parser.urdf_parser import URDFParser  # noqa: E402


@dataclass
class LinkRecord:
    name: str
    mass: float | None = None
    com: np.ndarray | None = None
    inertia: np.ndarray | None = None
    visuals: list[dict[str, Any]] = field(default_factory=list)
    collisions: list[dict[str, Any]] = field(default_factory=list)


@dataclass
class JointRecord:
    name: str
    joint_type: str
    parent: str
    child: str
    origin_xyz: np.ndarray
    origin_rpy: np.ndarray
    axis: np.ndarray
    lower: float | None = None
    upper: float | None = None
    effort: float | None = None
    velocity: float | None = None
    damping: float | None = None
    friction: float | None = None


@dataclass
class RobotRecord:
    name: str
    root_link: str
    links: dict[str, LinkRecord]
    joints: dict[str, JointRecord]


def _float_list(value: str | None, size: int, default: float = 0.0) -> np.ndarray:
    if value is None:
        return np.full(size, default, dtype=np.float64)
    parts = [float(v) for v in value.split()]
    if len(parts) != size:
        raise ValueError(f"Expected {size} floats, got {len(parts)} in {value!r}")
    return np.asarray(parts, dtype=np.float64)


def _origin_from_xml(elem) -> tuple[np.ndarray, np.ndarray]:
    if elem is None:
        return np.zeros(3, dtype=np.float64), np.zeros(3, dtype=np.float64)
    return (
        _float_list(elem.get("xyz"), 3, 0.0),
        _float_list(elem.get("rpy"), 3, 0.0),
    )


def _parse_geometry_xml(geom_elem) -> dict[str, Any]:
    if geom_elem is None:
        return {"type": "", "data": {}}
    for child in geom_elem:
        tag = child.tag.split("}")[-1]
        data: dict[str, Any] = dict(child.attrib)
        if tag == "mesh":
            data["filename"] = data.get("filename", "")
            data["scale"] = _float_list(data.get("scale", "1 1 1"), 3, 1.0).tolist()
        elif tag == "box":
            data["size"] = _float_list(data.get("size", "0 0 0"), 3, 0.0).tolist()
        elif tag in ("sphere", "cylinder", "capsule"):
            if "radius" in data:
                data["radius"] = float(data["radius"])
            if "length" in data:
                data["length"] = float(data["length"])
        return {"type": tag, "data": data}
    return {"type": "", "data": {}}


def _parse_shape_xml(elem) -> dict[str, Any]:
    xyz, rpy = _origin_from_xml(elem.find("origin"))
    geom = _parse_geometry_xml(elem.find("geometry"))
    return {
        "name": elem.get("name", ""),
        "origin_xyz": xyz,
        "origin_rpy": rpy,
        "geometry_type": geom["type"],
        "geometry_data": geom["data"],
    }


def parse_reference_xml(urdf_path: str) -> RobotRecord:
    root = ET.parse(urdf_path).getroot()
    robot_name = root.get("name", os.path.splitext(os.path.basename(urdf_path))[0])
    links: dict[str, LinkRecord] = {}
    joints: dict[str, JointRecord] = {}
    parent_map: dict[str, str] = {}

    for link_elem in root.findall("link"):
        name = link_elem.get("name", "")
        rec = LinkRecord(name=name)
        inertial = link_elem.find("inertial")
        if inertial is not None:
            mass_elem = inertial.find("mass")
            if mass_elem is not None:
                rec.mass = float(mass_elem.get("value", "0"))
            rec.com, _ = _origin_from_xml(inertial.find("origin"))
            inertia_elem = inertial.find("inertia")
            if inertia_elem is not None:
                ixx = float(inertia_elem.get("ixx", "0"))
                iyy = float(inertia_elem.get("iyy", "0"))
                izz = float(inertia_elem.get("izz", "0"))
                ixy = float(inertia_elem.get("ixy", "0"))
                ixz = float(inertia_elem.get("ixz", "0"))
                iyz = float(inertia_elem.get("iyz", "0"))
                rec.inertia = np.asarray(
                    [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]],
                    dtype=np.float64,
                )
        rec.visuals = [_parse_shape_xml(v) for v in link_elem.findall("visual")]
        rec.collisions = [_parse_shape_xml(c) for c in link_elem.findall("collision")]
        links[name] = rec

    for joint_elem in root.findall("joint"):
        parent_elem = joint_elem.find("parent")
        child_elem = joint_elem.find("child")
        parent = parent_elem.get("link", "") if parent_elem is not None else ""
        child = child_elem.get("link", "") if child_elem is not None else ""
        origin_xyz, origin_rpy = _origin_from_xml(joint_elem.find("origin"))
        axis_elem = joint_elem.find("axis")
        axis = _float_list(axis_elem.get("xyz"), 3, 0.0) if axis_elem is not None else np.asarray([1.0, 0.0, 0.0])
        limit_elem = joint_elem.find("limit")
        dynamics_elem = joint_elem.find("dynamics")
        name = joint_elem.get("name", "")
        joints[name] = JointRecord(
            name=name,
            joint_type=joint_elem.get("type", "fixed"),
            parent=parent,
            child=child,
            origin_xyz=origin_xyz,
            origin_rpy=origin_rpy,
            axis=axis,
            lower=float(limit_elem.get("lower")) if limit_elem is not None and limit_elem.get("lower") is not None else None,
            upper=float(limit_elem.get("upper")) if limit_elem is not None and limit_elem.get("upper") is not None else None,
            effort=float(limit_elem.get("effort")) if limit_elem is not None and limit_elem.get("effort") is not None else None,
            velocity=float(limit_elem.get("velocity")) if limit_elem is not None and limit_elem.get("velocity") is not None else None,
            damping=float(dynamics_elem.get("damping")) if dynamics_elem is not None and dynamics_elem.get("damping") is not None else None,
            friction=float(dynamics_elem.get("friction")) if dynamics_elem is not None and dynamics_elem.get("friction") is not None else None,
        )
        parent_map[child] = parent

    roots = set(links) - set(parent_map)
    root_link = sorted(roots)[0] if roots else (next(iter(links)) if links else "")
    return RobotRecord(robot_name, root_link, links, joints)


def parse_local(urdf_path: str) -> RobotRecord:
    model = URDFParser.parse(urdf_path)
    links: dict[str, LinkRecord] = {}
    for name, link in model.links.items():
        links[name] = LinkRecord(
            name=name,
            mass=link.inertial.mass if link.inertial is not None else None,
            com=np.asarray(link.inertial.com, dtype=np.float64) if link.inertial is not None else None,
            inertia=np.asarray(link.inertial.inertia, dtype=np.float64) if link.inertial is not None else None,
            visuals=[_shape_from_local(v) for v in link.visuals],
            collisions=[_shape_from_local(c) for c in link.collisions],
        )
    joints = {
        j.name: JointRecord(
            name=j.name,
            joint_type=j.joint_type,
            parent=j.parent,
            child=j.child,
            origin_xyz=np.asarray(j.origin_xyz, dtype=np.float64),
            origin_rpy=np.asarray(j.origin_rpy, dtype=np.float64),
            axis=np.asarray(j.axis, dtype=np.float64),
            lower=j.lower_limit,
            upper=j.upper_limit,
            effort=j.effort_limit,
            velocity=j.velocity_limit,
            damping=j.damping,
            friction=j.friction,
        )
        for j in model.joints
    }
    return RobotRecord(model.name, model.root_link, links, joints)


def _shape_from_local(shape) -> dict[str, Any]:
    return {
        "name": shape.name,
        "origin_xyz": np.asarray(shape.origin_xyz, dtype=np.float64),
        "origin_rpy": np.asarray(shape.origin_rpy, dtype=np.float64),
        "geometry_type": shape.geometry_type,
        "geometry_data": dict(shape.geometry_data),
    }


def _rpy_from_matrix(matrix: np.ndarray) -> np.ndarray:
    sy = -matrix[2, 0]
    cy = math.sqrt(max(0.0, 1.0 - sy * sy))
    if cy > 1.0e-12:
        roll = math.atan2(matrix[2, 1], matrix[2, 2])
        pitch = math.atan2(sy, cy)
        yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    else:
        roll = math.atan2(-matrix[1, 2], matrix[1, 1])
        pitch = math.atan2(sy, cy)
        yaw = 0.0
    return np.asarray([roll, pitch, yaw], dtype=np.float64)


def _origin_from_third_party(obj: Any) -> tuple[np.ndarray, np.ndarray]:
    origin = getattr(obj, "origin", None)
    if origin is None:
        return np.zeros(3, dtype=np.float64), np.zeros(3, dtype=np.float64)
    arr = np.asarray(origin, dtype=np.float64)
    if arr.shape == (4, 4):
        return arr[:3, 3], _rpy_from_matrix(arr[:3, :3])
    if arr.shape == (3,):
        return arr, np.zeros(3, dtype=np.float64)
    return np.zeros(3, dtype=np.float64), np.zeros(3, dtype=np.float64)


def parse_yourdfpy(urdf_path: str) -> RobotRecord:
    from yourdfpy import URDF

    robot = URDF.load(urdf_path)
    links_raw = getattr(robot, "links", None) or getattr(getattr(robot, "robot", None), "links", [])
    joints_raw = getattr(robot, "joints", None) or getattr(getattr(robot, "robot", None), "joints", [])
    return _parse_third_party_robot(getattr(robot, "name", ""), links_raw, joints_raw)


def parse_urdfpy(urdf_path: str) -> RobotRecord:
    from urdfpy import URDF

    robot = URDF.load(urdf_path)
    return _parse_third_party_robot(getattr(robot, "name", ""), robot.links, robot.joints)


def _parse_third_party_robot(name: str, links_raw: list[Any], joints_raw: list[Any]) -> RobotRecord:
    links: dict[str, LinkRecord] = {}
    joints: dict[str, JointRecord] = {}
    parent_map: dict[str, str] = {}

    for link in links_raw:
        link_name = getattr(link, "name", "")
        inertial = getattr(link, "inertial", None)
        mass = getattr(inertial, "mass", None) if inertial is not None else None
        com = None
        inertia = None
        if inertial is not None:
            com, _ = _origin_from_third_party(inertial)
            inertia_attr = getattr(inertial, "inertia", None)
            if inertia_attr is not None:
                inertia = np.asarray(inertia_attr, dtype=np.float64)
        links[link_name] = LinkRecord(
            name=link_name,
            mass=mass,
            com=com,
            inertia=inertia,
            visuals=[_shape_from_third_party(s) for s in getattr(link, "visuals", [])],
            collisions=[_shape_from_third_party(s) for s in getattr(link, "collisions", [])],
        )

    for joint in joints_raw:
        parent = getattr(joint, "parent", "")
        child = getattr(joint, "child", "")
        if not isinstance(parent, str):
            parent = getattr(parent, "name", "")
        if not isinstance(child, str):
            child = getattr(child, "name", "")
        xyz, rpy = _origin_from_third_party(joint)
        limit = getattr(joint, "limit", None)
        dynamics = getattr(joint, "dynamics", None)
        axis = getattr(joint, "axis", None)
        if axis is None:
            axis = np.asarray([1.0, 0.0, 0.0], dtype=np.float64)
        joints[getattr(joint, "name", "")] = JointRecord(
            name=getattr(joint, "name", ""),
            joint_type=getattr(joint, "joint_type", getattr(joint, "type", "fixed")),
            parent=parent,
            child=child,
            origin_xyz=xyz,
            origin_rpy=rpy,
            axis=np.asarray(axis, dtype=np.float64),
            lower=getattr(limit, "lower", None) if limit is not None else None,
            upper=getattr(limit, "upper", None) if limit is not None else None,
            effort=getattr(limit, "effort", None) if limit is not None else None,
            velocity=getattr(limit, "velocity", None) if limit is not None else None,
            damping=getattr(dynamics, "damping", None) if dynamics is not None else None,
            friction=getattr(dynamics, "friction", None) if dynamics is not None else None,
        )
        parent_map[child] = parent

    roots = set(links) - set(parent_map)
    root_link = sorted(roots)[0] if roots else (next(iter(links)) if links else "")
    return RobotRecord(name, root_link, links, joints)


def _shape_from_third_party(shape: Any) -> dict[str, Any]:
    xyz, rpy = _origin_from_third_party(shape)
    geom = getattr(shape, "geometry", None)
    geom_type = ""
    geom_data: dict[str, Any] = {}
    if geom is not None:
        for key in ("mesh", "box", "sphere", "cylinder", "capsule"):
            value = getattr(geom, key, None)
            if value is not None:
                geom_type = key
                for attr in ("filename", "scale", "size", "radius", "length"):
                    if hasattr(value, attr):
                        raw = getattr(value, attr)
                        geom_data[attr] = np.asarray(raw).tolist() if isinstance(raw, np.ndarray) else raw
                break
    return {
        "name": getattr(shape, "name", ""),
        "origin_xyz": xyz,
        "origin_rpy": rpy,
        "geometry_type": geom_type,
        "geometry_data": geom_data,
    }


def choose_reference(kind: str, urdf_path: str) -> tuple[str, RobotRecord]:
    if kind in ("auto", "yourdfpy") and importlib.util.find_spec("yourdfpy") is not None:
        return "yourdfpy", parse_yourdfpy(urdf_path)
    if kind == "yourdfpy":
        raise RuntimeError("yourdfpy is not installed")
    if kind in ("auto", "urdfpy") and importlib.util.find_spec("urdfpy") is not None:
        return "urdfpy", parse_urdfpy(urdf_path)
    if kind == "urdfpy":
        raise RuntimeError("urdfpy is not installed")
    return "xml", parse_reference_xml(urdf_path)


def _close(a: Any, b: Any, tol: float) -> bool:
    if a is None or b is None:
        return a is None and b is None
    return bool(np.allclose(np.asarray(a, dtype=np.float64), np.asarray(b, dtype=np.float64), atol=tol, rtol=tol))


def _scalar_close(a: float | None, b: float | None, tol: float, zero_missing_equivalent: bool = False) -> bool:
    if zero_missing_equivalent and ((a is None and b == 0.0) or (b is None and a == 0.0)):
        return True
    if a is None or b is None:
        return a is None and b is None
    return abs(float(a) - float(b)) <= tol


def compare_records(local: RobotRecord, ref: RobotRecord, tol: float, zero_missing_equivalent: bool = False) -> list[str]:
    diffs: list[str] = []
    if local.name != ref.name:
        diffs.append(f"robot.name: local={local.name!r}, ref={ref.name!r}")
    if local.root_link != ref.root_link:
        diffs.append(f"root_link: local={local.root_link!r}, ref={ref.root_link!r}")

    _compare_name_sets("links", set(local.links), set(ref.links), diffs)
    _compare_name_sets("joints", set(local.joints), set(ref.joints), diffs)

    for name in sorted(set(local.links) & set(ref.links)):
        l_link = local.links[name]
        r_link = ref.links[name]
        prefix = f"link[{name}]"
        if not _scalar_close(l_link.mass, r_link.mass, tol, zero_missing_equivalent):
            diffs.append(f"{prefix}.mass: local={l_link.mass}, ref={r_link.mass}")
        if not _close(l_link.com, r_link.com, tol):
            diffs.append(f"{prefix}.com: local={l_link.com}, ref={r_link.com}")
        if not _close(l_link.inertia, r_link.inertia, tol):
            diffs.append(f"{prefix}.inertia mismatch")
        _compare_shapes(f"{prefix}.visual", l_link.visuals, r_link.visuals, tol, diffs)
        _compare_shapes(f"{prefix}.collision", l_link.collisions, r_link.collisions, tol, diffs)

    for name in sorted(set(local.joints) & set(ref.joints)):
        l_joint = local.joints[name]
        r_joint = ref.joints[name]
        prefix = f"joint[{name}]"
        for field_name in ("joint_type", "parent", "child"):
            if getattr(l_joint, field_name) != getattr(r_joint, field_name):
                diffs.append(f"{prefix}.{field_name}: local={getattr(l_joint, field_name)!r}, ref={getattr(r_joint, field_name)!r}")
        for field_name in ("origin_xyz", "origin_rpy", "axis"):
            if not _close(getattr(l_joint, field_name), getattr(r_joint, field_name), tol):
                diffs.append(f"{prefix}.{field_name}: local={getattr(l_joint, field_name)}, ref={getattr(r_joint, field_name)}")
        for field_name in ("lower", "upper", "effort", "velocity", "damping", "friction"):
            if not _scalar_close(getattr(l_joint, field_name), getattr(r_joint, field_name), tol, zero_missing_equivalent):
                diffs.append(f"{prefix}.{field_name}: local={getattr(l_joint, field_name)}, ref={getattr(r_joint, field_name)}")
    return diffs


def _compare_name_sets(label: str, local: set[str], ref: set[str], diffs: list[str]) -> None:
    missing = sorted(ref - local)
    extra = sorted(local - ref)
    if missing:
        diffs.append(f"{label}.missing_in_local: {missing}")
    if extra:
        diffs.append(f"{label}.extra_in_local: {extra}")


def _compare_shapes(prefix: str, local: list[dict[str, Any]], ref: list[dict[str, Any]], tol: float, diffs: list[str]) -> None:
    if len(local) != len(ref):
        diffs.append(f"{prefix}.count: local={len(local)}, ref={len(ref)}")
    for idx, (l_shape, r_shape) in enumerate(zip(local, ref)):
        shape_prefix = f"{prefix}[{idx}]"
        for field_name in ("name", "geometry_type"):
            if l_shape.get(field_name, "") != r_shape.get(field_name, ""):
                diffs.append(f"{shape_prefix}.{field_name}: local={l_shape.get(field_name)!r}, ref={r_shape.get(field_name)!r}")
        for field_name in ("origin_xyz", "origin_rpy"):
            if not _close(l_shape.get(field_name), r_shape.get(field_name), tol):
                diffs.append(f"{shape_prefix}.{field_name}: local={l_shape.get(field_name)}, ref={r_shape.get(field_name)}")
        _compare_geometry_data(shape_prefix, l_shape.get("geometry_data", {}), r_shape.get("geometry_data", {}), tol, diffs)


def _compare_geometry_data(prefix: str, local: dict[str, Any], ref: dict[str, Any], tol: float, diffs: list[str]) -> None:
    for key in sorted(set(local) | set(ref)):
        if key not in local or key not in ref:
            diffs.append(f"{prefix}.geometry_data.{key}: local={local.get(key)!r}, ref={ref.get(key)!r}")
            continue
        l_val = local[key]
        r_val = ref[key]
        if isinstance(l_val, (list, tuple, np.ndarray)) or isinstance(r_val, (list, tuple, np.ndarray)):
            if not _close(l_val, r_val, tol):
                diffs.append(f"{prefix}.geometry_data.{key}: local={l_val!r}, ref={r_val!r}")
        elif isinstance(l_val, float) or isinstance(r_val, float):
            if not _scalar_close(float(l_val), float(r_val), tol):
                diffs.append(f"{prefix}.geometry_data.{key}: local={l_val!r}, ref={r_val!r}")
        elif l_val != r_val:
            diffs.append(f"{prefix}.geometry_data.{key}: local={l_val!r}, ref={r_val!r}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Compare local URDFParser with a reference parser")
    parser.add_argument("urdf", help="URDF file to compare")
    parser.add_argument("--reference", choices=("auto", "xml", "yourdfpy", "urdfpy"), default="auto")
    parser.add_argument("--tol", type=float, default=1.0e-6)
    parser.add_argument("--limit", type=int, default=80, help="Maximum number of diffs to print")
    parser.add_argument(
        "--zero-missing-equivalent",
        action="store_true",
        help="Treat a missing optional scalar in the reference parser as equivalent to local 0.0",
    )
    parser.add_argument("--strict", action="store_true", help="Exit non-zero when any diff is found")
    args = parser.parse_args()

    local = parse_local(args.urdf)
    ref_name, ref = choose_reference(args.reference, args.urdf)
    diffs = compare_records(local, ref, args.tol, args.zero_missing_equivalent)

    print(f"URDF: {args.urdf}")
    print(f"Reference parser: {ref_name}")
    print(f"Local: links={len(local.links)}, joints={len(local.joints)}, root={local.root_link}")
    print(f"Ref:   links={len(ref.links)}, joints={len(ref.joints)}, root={ref.root_link}")
    if not diffs:
        print("Parser comparison: PASSED")
        return 0

    print(f"Parser comparison: FAILED ({len(diffs)} diffs)")
    for diff in diffs[: args.limit]:
        print(f"  - {diff}")
    if len(diffs) > args.limit:
        print(f"  ... {len(diffs) - args.limit} more diffs not shown")
    return 1 if args.strict else 0


if __name__ == "__main__":
    raise SystemExit(main())
