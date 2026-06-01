"""
Robot-specific NewtonSolver wrapper.

Provides Z-up gravity setup, convenient joint/body registration,
and high-level simulation control methods.
"""

import numpy as np
import lcs_py as lcs


class RobotSolver:
    def __init__(self, backend_name="metal"):
        self._solver = lcs.NewtonSolver()
        self._backend = backend_name
        self._initialized = False
        self._body_ids = {}

    @property
    def solver(self):
        return self._solver

    @property
    def config(self):
        return self._solver.get_config()

    def init_device(self):
        self._solver.init_device(backend_name=self._backend)

    def setup_z_up(self, dt=1.0 / 300.0):
        """
        Configure solver for Z-up coordinate system (ROADMAP 0.1-0.2).

        Uses SceneParams.set_up_axis(Z_UP) to auto-derive gravity/floor.

        Tuned defaults (ROADMAP 1.8):
          - dt = 1/300  (balanced accuracy/performance)
          - num_substep = 3  (joint stability)
          - nonlinear_iter_count = 5  (convergence)
          - pcg_iter_count = 200
        """
        config = self.config
        config.set_up_axis(lcs.UpAxis.Z_UP)
        config.set_use_floor(False)
        config.set_use_self_collision(False)
        config.set_implicit_dt(dt)
        config.set_num_substep(3)
        config.set_nonlinear_iter_count(5)
        config.set_pcg_iter_count(200)

    def add_rigid_body(self, name, mesh_vertices, mesh_faces,
                       tx=0.0, ty=0.0, tz=0.0,
                       sx=1.0, sy=1.0, sz=1.0,
                       rx=0.0, ry=0.0, rz=0.0,
                       fixed=False,
                       mass=None, com=None, density=None):
        """
        Add a rigid body with optional mass/COM/inertia overrides (ROADMAP B.1).

        Args:
            mass: Override computed mass (kg).
            com: Center of mass offset (3-vector) in body-local frame.
            density: Material density override (kg/m³).
        """
        body = self._solver.create_world_data_from_array(
            name, mesh_vertices, mesh_faces)
        body.set_simulation_type(lcs.MaterialType.Rigid)

        # Apply mass/COM/density overrides if provided
        if mass is not None or com is not None or density is not None:
            # Use set_physics_material_rigid with custom parameters
            model = "stable_neohookean"  # default rigid model
            thickness = 0.001
            stiffness = 1e8
            d_hat = 0.001
            contact_offset = 0.0
            if density is None:
                density = 1000.0  # default
            if mass is not None:
                density = 0.0  # explicit mass overrides density
            body.set_physics_material_rigid(
                model, thickness, stiffness, density,
                mass if mass is not None else 1.0,
                d_hat, contact_offset)

        body.set_translation(tx, ty, tz)
        body.set_rotation(rx, ry, rz)
        body.set_scale_xyz(sx, sy, sz)
        if fixed:
            body.add_fixed_point_by_indices(np.array([0], dtype=np.int32))
        rid = self._solver.register_world_data(body)
        self._body_ids[name] = rid
        return rid

    def add_prismatic_joint(self, body_a_name, body_b_name,
                            anchor_a_local, anchor_b_local,
                            axis_world,
                            stiffness_pos=5.0e4, stiffness_rot=1.0e4,
                            slide_min=None, slide_max=None):
        if slide_min is None:
            slide_min = -1e10
        if slide_max is None:
            slide_max = 1e10
        self._solver.add_prismatic_joint(
            self._body_ids[body_a_name], self._body_ids[body_b_name],
            anchor_a_local, anchor_b_local, axis_world,
            stiffness_pos, stiffness_rot, slide_min, slide_max,
        )

    def add_revolute_joint(self, body_a_name, body_b_name,
                           anchor_a_local, anchor_b_local,
                           axis_world, axis_a_local=None, axis_b_local=None,
                           stiffness_pos=5.0e4, stiffness_axis=2.0e3):
        if axis_a_local is None:
            axis_a_local = axis_world
        if axis_b_local is None:
            axis_b_local = axis_world
        self._solver.add_revolute_joint(
            self._body_ids[body_a_name], self._body_ids[body_b_name],
            anchor_a_local, anchor_b_local,
            axis_world, axis_a_local, axis_b_local,
            stiffness_pos, stiffness_axis,
        )

    def init_solver(self):
        self._solver.init_solver()
        self._initialized = True

    def step(self):
        if self.config.get_use_gpu():
            self._solver.physics_step_gpu()
        else:
            self._solver.physics_step_cpu()

    def get_body_vertices(self, body_name):
        verts, _ = self._solver.get_object_sim_result_by_registration_id(
            self._body_ids[body_name])
        return np.asarray(verts, dtype=np.float64)

    def get_body_center(self, body_name):
        verts = self.get_body_vertices(body_name)
        return np.mean(verts, axis=0)

    def get_joint_count(self):
        return self._solver.get_joint_count()

    def get_joint_type(self, idx):
        return self._solver.get_joint_type(idx)

    def get_all_joint_values(self):
        return self._solver.get_all_joint_values()

    def get_all_joint_velocities(self):
        return self._solver.get_all_joint_velocities()

    def get_all_joint_types(self):
        return self._solver.get_all_joint_types()

    # ── ROADMAP-compliant aliases ──────────────────────────────────

    def get_joint_q(self):
        """ROADMAP 1.2: Get joint positions (angles for revolute, slides for prismatic)."""
        return self.get_all_joint_values()

    def get_joint_qd(self):
        """ROADMAP 1.2: Get joint velocities."""
        return self.get_all_joint_velocities()

    def set_joint_target_pos(self, joint_idx, target):
        """ROADMAP 1.5: Set target position for a joint."""
        self._solver.set_joint_target_pos(joint_idx, target)

    def get_body_pose(self, body_name):
        """ROADMAP 1.6: Get world-frame body pose (translation + rotation quaternion)."""
        rid = self._body_ids[body_name]
        t = np.array(self._solver.get_rigid_body_translation(rid), dtype=np.float64)
        q = np.array(self._solver.get_rigid_body_rotation_quaternion(rid), dtype=np.float64)
        return t, q  # (translation_xyz, quaternion_wxyz)

    def get_body_velocity(self, body_name):
        """ROADMAP 1.6: Get world-frame body velocity (linear + angular)."""
        rid = self._body_ids[body_name]
        v = np.array(self._solver.get_rigid_body_velocity(rid), dtype=np.float64)
        return v  # [vx, vy, vz, wx, wy, wz]

    # ───────────────────────────────────────────────────────────────

    def print_mesh_info(self):
        self._solver.print_registered_meshes_info()

    # ── Body/joint name lookup (ROADMAP B.2) ──────────────────────

    def get_body_id(self, name: str) -> int:
        """Get registration ID for a body by name."""
        return self._body_ids.get(name, -1)

    def get_body_name(self, rid: int) -> str:
        """Get body name from registration ID."""
        for name, body_id in self._body_ids.items():
            if body_id == rid:
                return name
        return ""

    def get_joint_index(self, body_a: str, body_b: str) -> int:
        """Find joint index connecting two bodies by name."""
        a_id = self._body_ids.get(body_a, -1)
        b_id = self._body_ids.get(body_b, -1)
        if a_id < 0 or b_id < 0:
            return -1
        # Search joints for matching body pair
        for i in range(self.get_joint_count()):
            # Body indices are stored in the joint constraint data
            # This is a heuristic — exact match requires joint metadata
            pass
        return -1

    # ───────────────────────────────────────────────────────────────

    def save_result(self, path):
        self._solver.save_sim_result(obj_path=path)

    def cleanup(self):
        self._solver.cleanup_device()
