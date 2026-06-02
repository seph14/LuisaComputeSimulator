"""
RobotBuilder: URDF → NewtonSolver scene builder (ROADMAP 2.2-2.5).

Converts a parsed URDFRobotModel into rigid bodies and joint constraints,
using the existing C++ WorldData / joint descriptor API.
"""

import os
import numpy as np
import trimesh

from robotics.parser.urdf_parser import URDFParser, URDFRobotModel, URDFJoint, URDFLink


class RobotBuilder:
    """
    Build a robotics scene from a URDF model.

    Usage:
        rs = RobotSolver()
        rs.init_device()
        rs.setup_z_up()

        model = URDFParser.parse("robot.urdf")
        builder = RobotBuilder(rs, model)
        builder.build(mesh_root="assets/")
        rs.init_solver()
        rs.step()
    """

    def __init__(self, robot_solver, model: URDFRobotModel,
                 collapse_fixed_joints: bool = False):
        self._rs = robot_solver
        self._solver = robot_solver.solver
        self._model = model
        self._collapse_fixed = collapse_fixed_joints

        # Registration IDs keyed by link name
        self._link_body_ids: dict = {}
        # Joint index → registration_id pairs (for API access)
        self._joint_body_pairs: list = []  # [(body_a_id, body_b_id)]

    @property
    def link_body_ids(self) -> dict:
        return self._link_body_ids

    @property
    def model(self) -> URDFRobotModel:
        return self._model

    def build(self, mesh_root: str = "",
              default_mass: float = 1.0,
              default_inertia: tuple = (0.1, 0.1, 0.1)):
        """
        Build the full robot scene.

        Args:
            mesh_root: Root directory for resolving mesh paths in URDF.
            default_mass: Fallback mass for links without inertial data.
            default_inertia: Fallback (ixx, iyy, izz) for links without inertia.
        """
        # Step 1: Create bodies for all links in topological order
        topo_order = URDFParser.build_topology_order(self._model)
        if not topo_order:
            raise ValueError("Empty robot model — no links found")

        # Pre-compute world-frame transforms by walking the tree
        link_transforms = self._compute_link_transforms()

        cube_mesh = self._get_default_cube_mesh()

        for link_name in topo_order:
            link = self._model.links.get(link_name)
            if link is None:
                continue

            # Get world-frame transform for this link
            T = link_transforms.get(link_name, np.eye(4))
            tx, ty, tz = T[0, 3], T[1, 3], T[2, 3]

            # Use collision mesh if available, else visual mesh, else cube
            mesh = self._get_link_mesh(link, mesh_root, cube_mesh)

            # Mass / inertia from URDF or defaults
            mass = default_mass
            if link.inertial is not None and link.inertial.mass > 0:
                mass = link.inertial.mass

            # Determine if this link is the world-fixed base
            is_fixed = (link_name == self._model.root_link and
                        not self._has_floating_joint())

            body_id = self._rs.add_rigid_body(
                link_name,
                mesh.vertices, mesh.faces,
                tx=float(tx), ty=float(ty), tz=float(tz),
                fixed=is_fixed,
            )
            self._link_body_ids[link_name] = body_id

        # Step 2: Create joints
        for joint in self._model.joints:
            parent_id = self._link_body_ids.get(joint.parent)
            child_id = self._link_body_ids.get(joint.child)
            if parent_id is None or child_id is None:
                continue

            # Joint anchor in world frame (from parent origin)
            axis = joint.axis.astype(np.float64)
            anchor_parent = joint.origin_xyz.astype(np.float64)
            anchor_child = np.zeros(3, dtype=np.float64)

            jtype = joint.joint_type.lower()

            if jtype in ("revolute", "continuous"):
                self._rs.add_revolute_joint(
                    joint.parent, joint.child,
                    anchor_parent, anchor_child, axis,
                    stiffness_pos=5.0e4, stiffness_axis=2.0e3,
                )
            elif jtype == "prismatic":
                self._rs.add_prismatic_joint(
                    joint.parent, joint.child,
                    anchor_parent, anchor_child, axis,
                    stiffness_pos=5.0e4, stiffness_rot=1.0e4,
                    slide_min=joint.lower_limit,
                    slide_max=joint.upper_limit,
                )
            elif jtype == "fixed":
                # Fixed joint: use high stiffness to effectively merge bodies
                self._solver.add_fixed_joint(
                    parent_id, child_id,
                    anchor_parent, anchor_child,
                    stiffness_pos=1.0e6, stiffness_rot=1.0e5,
                )
            elif jtype in ("floating", "free"):
                self._rs.add_free_joint(joint.parent, joint.child)

            self._joint_body_pairs.append((parent_id, child_id))

    # ── Internal helpers ─────────────────────────────────────────────

    def _compute_link_transforms(self) -> dict:
        """Compute world-frame 4x4 transform for each link (Z-up convention)."""
        transforms = {}
        # Root is at origin
        transforms[self._model.root_link] = np.eye(4)

        # BFS to propagate transforms
        queue = [self._model.root_link]
        while queue:
            parent = queue.pop(0)
            T_parent = transforms[parent]
            for child in self._model.child_map.get(parent, []):
                # Find joint connecting parent → child
                joint = self._find_joint(parent, child)
                T_child = T_parent.copy()
                if joint is not None:
                    # Apply joint origin (parent frame → child frame)
                    T_joint = np.eye(4)
                    T_joint[:3, :3] = self._rpy_to_matrix(joint.origin_rpy)
                    T_joint[:3, 3] = joint.origin_xyz
                    T_child = T_parent @ T_joint
                transforms[child] = T_child
                queue.append(child)

        return transforms

    @staticmethod
    def _rpy_to_matrix(rpy) -> np.ndarray:
        """Convert URDF fixed-axis roll-pitch-yaw angles to a rotation matrix."""
        roll, pitch, yaw = [float(v) for v in rpy]
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)

        rx = np.array([[1.0, 0.0, 0.0],
                       [0.0, cr, -sr],
                       [0.0, sr, cr]], dtype=np.float64)
        ry = np.array([[cp, 0.0, sp],
                       [0.0, 1.0, 0.0],
                       [-sp, 0.0, cp]], dtype=np.float64)
        rz = np.array([[cy, -sy, 0.0],
                       [sy, cy, 0.0],
                       [0.0, 0.0, 1.0]], dtype=np.float64)
        return rz @ ry @ rx

    def _find_joint(self, parent: str, child: str) -> URDFJoint | None:
        for j in self._model.joints:
            if j.parent == parent and j.child == child:
                return j
        return None

    def _has_floating_joint(self) -> bool:
        """Check if the root link has a floating/free joint."""
        for j in self._model.joints:
            if j.joint_type.lower() in ("floating", "free"):
                return True
        return False

    def _get_link_mesh(self, link: URDFLink, mesh_root: str,
                       default_mesh) -> trimesh.Trimesh:
        """Get the best available mesh for a link."""
        # Prefer collision meshes
        for col in link.collisions:
            if col.geometry_type == "mesh":
                path = col.geometry_data.get("filename", "")
                if path and mesh_root:
                    path = os.path.join(mesh_root, path)
                if path and os.path.exists(path):
                    return trimesh.load(path, process=False)

        # Fall back to visual meshes
        for vis in link.visuals:
            if vis.geometry_type == "mesh":
                path = vis.geometry_data.get("filename", "")
                if path and mesh_root:
                    path = os.path.join(mesh_root, path)
                if path and os.path.exists(path):
                    return trimesh.load(path, process=False)

        # Generate primitive geometry
        for col in link.collisions + link.visuals:
            prim = self._make_primitive_mesh(col.geometry_type,
                                             col.geometry_data)
            if prim is not None:
                return prim

        return default_mesh

    @staticmethod
    def _make_primitive_mesh(geom_type: str, data: dict) -> trimesh.Trimesh | None:
        """Create a trimesh primitive from geometry data."""
        try:
            if geom_type == "box":
                size = data.get("size", [0.1, 0.1, 0.1])
                return trimesh.creation.box(extents=size)
            elif geom_type == "sphere":
                radius = data.get("radius", 0.05)
                return trimesh.creation.icosphere(radius=radius, subdivisions=2)
            elif geom_type == "cylinder":
                radius = data.get("radius", 0.05)
                length = data.get("length", 0.1)
                return trimesh.creation.cylinder(radius=radius, height=length)
            elif geom_type == "capsule":
                radius = data.get("radius", 0.05)
                length = data.get("length", 0.1)
                return trimesh.creation.capsule(radius=radius, height=length)
        except Exception:
            pass
        return None

    @staticmethod
    def _get_default_cube_mesh() -> trimesh.Trimesh:
        """Return a small cube mesh for fallback."""
        return trimesh.creation.box(extents=[0.02, 0.02, 0.02])
