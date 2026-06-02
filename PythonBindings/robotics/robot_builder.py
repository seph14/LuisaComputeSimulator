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
                 collapse_fixed_joints: bool = False,
                 fixed_base: bool = True):
        self._rs = robot_solver
        self._solver = robot_solver.solver
        self._model = model
        self._collapse_fixed = collapse_fixed_joints
        self._fixed_base = fixed_base

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
              default_inertia: tuple = (0.1, 0.1, 0.1),
              base_translation: tuple = (0.0, 0.0, 0.0),
              swap_yz: bool = False,
              floor_height: float | None = None,
              floor_normal: tuple = (0.0, 0.0, 1.0),
              floor_clearance: float = 0.01):
        """
        Build the full robot scene.

        Args:
            mesh_root: Root directory for resolving mesh paths in URDF.
            default_mass: Fallback mass for links without inertial data.
            default_inertia: Fallback (ixx, iyy, izz) for links without inertia.
            base_translation: World-space translation applied to the root link.
            swap_yz: Swap URDF Y/Z axes before registering bodies and joints.
            floor_height: If set, lift the robot along floor_normal so its
                minimum signed floor coordinate is floor_height + floor_clearance.
            floor_normal: Plane normal used by floor_height/auto-lift.
            floor_clearance: Clearance used with floor_height.
        """
        # Step 1: Create bodies for all links in topological order
        topo_order = URDFParser.build_topology_order(self._model)
        if not topo_order:
            raise ValueError("Empty robot model — no links found")

        # Pre-compute world-frame transforms by walking the tree
        link_transforms = self._compute_link_transforms()
        axis_xform = self._axis_transform_matrix(swap_yz)

        cube_mesh = self._get_default_cube_mesh()
        body_specs = []
        floor_n = np.asarray(floor_normal, dtype=np.float64)
        n_norm = np.linalg.norm(floor_n)
        if n_norm <= 0.0:
            floor_n = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
        else:
            floor_n = floor_n / n_norm
        min_floor_coord = np.inf

        for link_name in topo_order:
            link = self._model.links.get(link_name)
            if link is None:
                continue

            # Get world-frame transform for this link
            T = self._apply_axis_transform(link_transforms.get(link_name, np.eye(4)), axis_xform)

            # Use collision mesh if available, else visual mesh, else cube
            mesh = self._get_link_mesh(link, mesh_root, cube_mesh)
            mesh = self._apply_mesh_axis_transform(mesh, axis_xform)

            verts_h = np.column_stack([np.asarray(mesh.vertices, dtype=np.float64), np.ones(len(mesh.vertices))])
            world_verts = (T @ verts_h.T).T[:, :3]
            if len(world_verts):
                min_floor_coord = min(min_floor_coord, float(np.min(world_verts @ floor_n)))

            # Mass / inertia from URDF or defaults
            mass = default_mass
            if link.inertial is not None and link.inertial.mass > 0:
                mass = link.inertial.mass

            # Determine if this link is the world-fixed base
            is_fixed = (self._fixed_base and link_name == self._model.root_link and
                        not self._has_floating_joint())

            body_specs.append((link_name, mesh, T, mass, is_fixed))

        lift_vec = np.zeros(3, dtype=np.float64)
        if floor_height is not None and np.isfinite(min_floor_coord):
            min_floor_coord += float(np.dot(np.asarray(base_translation, dtype=np.float64), floor_n))
            lift = max(0.0, float(floor_height) + float(floor_clearance) - min_floor_coord)
            lift_vec = floor_n * lift

        for link_name, mesh, T, mass, is_fixed in body_specs:
            tx = T[0, 3] + float(base_translation[0])
            ty = T[1, 3] + float(base_translation[1])
            tz = T[2, 3] + float(base_translation[2])
            tx += float(lift_vec[0])
            ty += float(lift_vec[1])
            tz += float(lift_vec[2])
            rx, ry, rz = self._matrix_to_lcs_euler(T[:3, :3])

            # Pass URDF mass/COM if available (P3.1)
            kw_mass = {}
            if mass is not None and mass > 0:
                kw_mass["mass"] = float(mass)
            if link.inertial is not None:
                if link.inertial.com is not None and np.any(link.inertial.com != 0):
                    kw_mass["com"] = np.array(link.inertial.com, dtype=np.float64)

            body_id = self._rs.add_rigid_body(
                link_name,
                mesh.vertices, mesh.faces,
                tx=float(tx), ty=float(ty), tz=float(tz),
                rx=float(rx), ry=float(ry), rz=float(rz),
                fixed=is_fixed,
                **kw_mass,
            )
            self._link_body_ids[link_name] = body_id

        # Step 2: Create joints
        for joint in self._model.joints:
            parent_id = self._link_body_ids.get(joint.parent)
            child_id = self._link_body_ids.get(joint.child)
            if parent_id is None or child_id is None:
                continue

            # Joint anchor in world frame (from parent origin)
            axis = axis_xform @ joint.axis.astype(np.float64)
            anchor_parent = axis_xform @ joint.origin_xyz.astype(np.float64)
            anchor_child = np.zeros(3, dtype=np.float64)

            jtype = joint.joint_type.lower()

            if jtype in ("revolute", "continuous"):
                # Pass URDF angle limits if <limit> element was present (P3.1)
                # Use joint.has_limits to distinguish "limit=0" from "no limit element"
                lower_angle = joint.lower_limit if joint.has_limits else -1e10
                upper_angle = joint.upper_limit if joint.has_limits else 1e10
                self._rs.add_revolute_joint(
                    joint.parent, joint.child,
                    anchor_parent, anchor_child, axis,
                    stiffness_pos=5.0e4, stiffness_axis=2.0e3,
                    lower_angle=lower_angle, upper_angle=upper_angle,
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

    @staticmethod
    def _axis_transform_matrix(swap_yz: bool) -> np.ndarray:
        if not swap_yz:
            return np.eye(3, dtype=np.float64)
        return np.array([[1.0, 0.0, 0.0],
                         [0.0, 0.0, 1.0],
                         [0.0, 1.0, 0.0]], dtype=np.float64)

    @staticmethod
    def _apply_axis_transform(T: np.ndarray, axis_xform: np.ndarray) -> np.ndarray:
        result = np.eye(4, dtype=np.float64)
        result[:3, :3] = axis_xform @ T[:3, :3] @ axis_xform.T
        result[:3, 3] = axis_xform @ T[:3, 3]
        return result

    @staticmethod
    def _apply_mesh_axis_transform(mesh: trimesh.Trimesh, axis_xform: np.ndarray) -> trimesh.Trimesh:
        if np.allclose(axis_xform, np.eye(3)):
            return mesh
        result = mesh.copy()
        result.vertices = (axis_xform @ np.asarray(result.vertices, dtype=np.float64).T).T
        if np.linalg.det(axis_xform) < 0.0:
            result.faces = np.asarray(result.faces)[:, ::-1]
        return result

    @staticmethod
    def _matrix_to_rpy(matrix) -> np.ndarray:
        """Convert a rotation matrix to URDF fixed-axis roll-pitch-yaw angles."""
        m = np.asarray(matrix, dtype=np.float64)
        sy = -m[2, 0]
        cy = np.sqrt(max(0.0, 1.0 - sy * sy))
        if cy > 1.0e-12:
            roll = np.arctan2(m[2, 1], m[2, 2])
            pitch = np.arctan2(sy, cy)
            yaw = np.arctan2(m[1, 0], m[0, 0])
        else:
            roll = np.arctan2(-m[1, 2], m[1, 1])
            pitch = np.arctan2(sy, cy)
            yaw = 0.0
        return np.array([roll, pitch, yaw], dtype=np.float64)

    @staticmethod
    def _matrix_to_lcs_euler(matrix) -> np.ndarray:
        """Convert a rotation matrix to the Euler order used by WorldData.set_rotation."""
        m = np.asarray(matrix, dtype=np.float64)
        sy = m[0, 2]
        cy = np.sqrt(max(0.0, 1.0 - sy * sy))
        if cy > 1.0e-12:
            rx = np.arctan2(-m[1, 2], m[2, 2])
            ry = np.arctan2(sy, cy)
            rz = np.arctan2(-m[0, 1], m[0, 0])
        else:
            rx = np.arctan2(m[2, 1], m[1, 1])
            ry = np.arctan2(sy, cy)
            rz = 0.0
        return np.array([rx, ry, rz], dtype=np.float64)

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
                    try:
                        mesh = trimesh.load(path, process=False)
                        return self._apply_shape_transform(mesh, col)
                    except Exception:
                        pass

        # Fall back to visual meshes
        for vis in link.visuals:
            if vis.geometry_type == "mesh":
                path = vis.geometry_data.get("filename", "")
                if path and mesh_root:
                    path = os.path.join(mesh_root, path)
                if path and os.path.exists(path):
                    try:
                        mesh = trimesh.load(path, process=False)
                        return self._apply_shape_transform(mesh, vis)
                    except Exception:
                        pass

        # Generate primitive geometry
        for col in link.collisions + link.visuals:
            prim = self._make_primitive_mesh(col.geometry_type,
                                             col.geometry_data)
            if prim is not None:
                return self._apply_shape_transform(prim, col)

        return default_mesh

    @staticmethod
    def _apply_shape_transform(mesh: trimesh.Trimesh, shape) -> trimesh.Trimesh:
        """Bake a URDF visual/collision origin and mesh scale into a link-local mesh."""
        result = mesh.copy()
        scale = shape.geometry_data.get("scale") if shape.geometry_type == "mesh" else None
        if scale is not None:
            scale_vec = np.asarray(scale, dtype=np.float64)
            result.vertices = np.asarray(result.vertices, dtype=np.float64) * scale_vec

        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = RobotBuilder._rpy_to_matrix(shape.origin_rpy)
        T[:3, 3] = np.asarray(shape.origin_xyz, dtype=np.float64)
        result.apply_transform(T)
        return result

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
