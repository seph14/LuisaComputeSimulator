"""
Robot-specific NewtonSolver wrapper.

Provides Z-up gravity setup, convenient joint/body registration,
and high-level simulation control methods.
"""

import numpy as np
import lcs_py as lcs

from robotics.solver.world_meta import WorldMeta, BodyMeta, JointMeta


class RobotSolver:
    def __init__(self, backend_name="metal"):
        self._solver = lcs.NewtonSolver()
        self._backend = backend_name
        self._initialized = False
        self._body_ids = {}
        # Joint metadata: maps (parent_name, child_name) -> (joint_index, joint_type)
        self._joint_index_map: dict = {}
        self._joint_names: list = []
        # Multi-world metadata (P2.1)
        self._world_meta: WorldMeta = WorldMeta()

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
        self._record_joint(body_a_name, body_b_name, "prismatic")

    def add_revolute_joint(self, body_a_name, body_b_name,
                           anchor_a_local, anchor_b_local,
                           axis_world, axis_a_local=None, axis_b_local=None,
                           stiffness_pos=5.0e4, stiffness_axis=2.0e3,
                           lower_angle=-1e10, upper_angle=1e10):
        if axis_a_local is None:
            axis_a_local = axis_world
        if axis_b_local is None:
            axis_b_local = axis_world
        self._solver.add_revolute_joint(
            self._body_ids[body_a_name], self._body_ids[body_b_name],
            anchor_a_local, anchor_b_local,
            axis_world, axis_a_local, axis_b_local,
            stiffness_pos, stiffness_axis, lower_angle, upper_angle)
        self._record_joint(body_a_name, body_b_name, "revolute")

    def add_ball_joint(self, body_a_name, body_b_name,
                       anchor_a_local, anchor_b_local,
                       stiffness_pos=5.0e4):
        """Add a ball (spherical) joint: constrains anchors to coincide, free rotation."""
        self._solver.add_ball_joint(
            self._body_ids[body_a_name], self._body_ids[body_b_name],
            anchor_a_local, anchor_b_local,
            stiffness_pos,
        )
        self._record_joint(body_a_name, body_b_name, "ball")

    def add_free_joint(self, body_a_name, body_b_name):
        """Add a free (floating) joint: no constraint (placeholder for floating base)."""
        self._solver.add_free_joint(
            self._body_ids[body_a_name], self._body_ids[body_b_name],
        )
        self._record_joint(body_a_name, body_b_name, "free")

    def _record_joint(self, body_a_name, body_b_name, jtype):
        """Record joint metadata for name-based lookup."""
        self._record_joint_returning_idx(body_a_name, body_b_name, jtype)

    def _record_joint_returning_idx(self, body_a_name, body_b_name, jtype):
        """Record joint metadata and return the assigned joint index."""
        idx = len(self._joint_names)
        pair = (body_a_name, body_b_name)
        self._joint_index_map[pair] = (idx, jtype)
        self._joint_names.append(f"{body_a_name}_{body_b_name}_{jtype}")
        return idx

    # ── Multi-world replication (P2.2) ───────────────────────────────

    def replicate(self, body_specs: list, joint_specs: list,
                  world_count: int, spacing: tuple = (0.0, 2.0, 0.0)):
        """
        Register world_count-1 additional copies of all bodies and joints,
        each offset by `spacing` from the previous world.

        Must be called AFTER world 0 bodies/joints are registered
        and BEFORE init_solver().

        Args:
            body_specs: list of dicts with keys:
                name, vertices, faces, tx, ty, tz, rx, ry, rz, sx, sy, sz,
                fixed, mass, com, density
            joint_specs: list of dicts with keys:
                parent_name, child_name, joint_type, kwargs
            world_count: Total number of worlds (including already-built world 0).
            spacing: (dx, dy, dz) offset between consecutive worlds.
        """
        dx, dy, dz = spacing

        # Compute DOF per world
        dof_per_world = 0
        for js in joint_specs:
            jtype = js["joint_type"]
            if jtype not in ("fixed", "free"):
                dof_per_world += 1

        # Populate world 0 metadata from existing _body_ids and _joint_index_map
        wm = WorldMeta()
        wm.world_count = world_count
        wm.body_count_per_world = len(body_specs)
        wm.joint_count_per_world = len(joint_specs)
        wm.dof_per_world = dof_per_world

        # Register world 0 metadata
        for bs in body_specs:
            name = bs["name"]
            rid = self._body_ids.get(name, -1)
            bm = BodyMeta(name=name, world_id=0, registration_id=rid,
                          fixed=bs.get("fixed", False))
            wm.body_map.setdefault(0, {})[name] = bm
            wm.rid_to_body[rid] = bm

        # Map joint specs to recorded indices
        dof_idx = 0
        for ji, js in enumerate(joint_specs):
            pname = js["parent_name"]
            cname = js["child_name"]
            jtype = js["joint_type"]
            pair = (pname, cname)
            rev_pair = (cname, pname)
            if pair in self._joint_index_map:
                jidx, _ = self._joint_index_map[pair]
            elif rev_pair in self._joint_index_map:
                jidx, _ = self._joint_index_map[rev_pair]
            else:
                # Fallback: joint not in world 0's _joint_index_map (e.g. fixed joints
                # added directly via _solver bypassing _record_joint). Use positional offset.
                jidx = self._joint_index_map.get(
                    list(self._joint_index_map.keys())[ji], (ji, "")
                )[0] if ji < len(self._joint_index_map) else ji
                print(f"  WARNING: joint ({pname}, {cname}) not in index map; "
                      f"using fallback index {jidx}")
            jm = JointMeta(
                name=f"{pname}_{cname}_{jtype}",
                parent_name=pname, child_name=cname,
                joint_type=jtype, world_id=0,
                joint_index=jidx,
                dof_index=(dof_idx if jtype not in ("fixed", "free") else -1),
            )
            if jtype not in ("fixed", "free"):
                dof_idx += 1
            wm.joint_map.setdefault(0, {})[jm.name] = jm
            wm.jidx_to_joint[jidx] = jm

        # Register worlds 1..world_count-1
        for w in range(1, world_count):
            offset_x = w * dx
            offset_y = w * dy
            offset_z = w * dz
            dof_idx_w = 0

            # Clone bodies
            for bs in body_specs:
                name = bs["name"]
                world_name = f"world_{w}/{name}"
                rid = self.add_rigid_body(
                    world_name,
                    bs["vertices"], bs["faces"],
                    bs["tx"] + offset_x,
                    bs["ty"] + offset_y,
                    bs["tz"] + offset_z,
                    bs.get("sx", 1.0), bs.get("sy", 1.0), bs.get("sz", 1.0),
                    bs.get("rx", 0.0), bs.get("ry", 0.0), bs.get("rz", 0.0),
                    fixed=bs.get("fixed", False),
                    mass=bs.get("mass"), com=bs.get("com"),
                    density=bs.get("density"),
                )
                bm = BodyMeta(name=name, world_id=w, registration_id=rid,
                              fixed=bs.get("fixed", False))
                wm.body_map.setdefault(w, {})[name] = bm
                wm.rid_to_body[rid] = bm

            # Clone joints
            for js in joint_specs:
                pname = js["parent_name"]
                cname = js["child_name"]
                jtype = js["joint_type"]
                kwargs = js.get("kwargs", {})

                parent_rid = wm.body_map[w][pname].registration_id
                child_rid = wm.body_map[w][cname].registration_id

                if jtype == "fixed":
                    self._solver.add_fixed_joint(
                        parent_rid, child_rid,
                        kwargs.get("anchor_a", np.zeros(3, dtype=np.float32)),
                        kwargs.get("anchor_b", np.zeros(3, dtype=np.float32)),
                        kwargs.get("stiffness_pos", 1.0e6),
                        kwargs.get("stiffness_rot", 1.0e5),
                    )
                elif jtype == "prismatic":
                    self._solver.add_prismatic_joint(
                        parent_rid, child_rid,
                        kwargs["anchor_a"], kwargs["anchor_b"],
                        kwargs["axis"],
                        kwargs.get("stiffness_pos", 5.0e4),
                        kwargs.get("stiffness_rot", 1.0e4),
                        kwargs.get("slide_min", -1e10),
                        kwargs.get("slide_max", 1e10),
                    )
                elif jtype == "revolute":
                    self._solver.add_revolute_joint(
                        parent_rid, child_rid,
                        kwargs["anchor_a"], kwargs["anchor_b"],
                        kwargs["axis"],
                        kwargs.get("axis_a", kwargs["axis"]),
                        kwargs.get("axis_b", kwargs["axis"]),
                        kwargs.get("stiffness_pos", 5.0e4),
                        kwargs.get("stiffness_axis", 2.0e3),
                    )
                elif jtype == "ball":
                    self._solver.add_ball_joint(
                        parent_rid, child_rid,
                        kwargs["anchor_a"], kwargs["anchor_b"],
                        kwargs.get("stiffness_pos", 5.0e4),
                    )
                elif jtype == "free":
                    self._solver.add_free_joint(parent_rid, child_rid)

                joint_name = f"{pname}_{cname}_{jtype}"
                # _record_joint was already called for world 0;
                # for worlds 1+, record via returned index
                jidx = self._record_joint_returning_idx(pname, cname, jtype)
                jm = JointMeta(
                    name=joint_name,
                    parent_name=pname, child_name=cname,
                    joint_type=jtype, world_id=w,
                    joint_index=jidx,
                    dof_index=(dof_idx_w if jtype not in ("fixed", "free") else -1),
                )
                if jtype not in ("fixed", "free"):
                    dof_idx_w += 1
                wm.joint_map.setdefault(w, {})[joint_name] = jm
                wm.jidx_to_joint[jidx] = jm

        self._world_meta = wm
        print(f"  Replicated: {world_count} worlds, "
              f"{world_count * len(body_specs)} bodies, "
              f"{world_count * len(joint_specs)} joints")

    # ── Initial state save/restore (P2.4) ──────────────────────────

    def save_initial_state(self):
        """Snapshot current body poses and joint q for reset support.
        Call AFTER init_solver(), BEFORE first step()."""
        wm = self._world_meta
        if wm.world_count == 0:
            # Single-world mode: save from _body_ids
            for name, rid in self._body_ids.items():
                t = np.array(self._solver.get_rigid_body_translation(rid),
                             dtype=np.float64)
                q = np.array(self._solver.get_rigid_body_rotation_quaternion(rid),
                             dtype=np.float64)
                wm.initial_body_poses[rid] = np.concatenate([t, q])
            joint_vals = self.get_all_joint_values()
            for jidx in range(len(joint_vals)):
                wm.initial_joint_q[jidx] = float(joint_vals[jidx])
        else:
            for rid in wm.rid_to_body:
                t = np.array(self._solver.get_rigid_body_translation(rid),
                             dtype=np.float64)
                q = np.array(self._solver.get_rigid_body_rotation_quaternion(rid),
                             dtype=np.float64)
                wm.initial_body_poses[rid] = np.concatenate([t, q])
            joint_vals = self.get_all_joint_values()
            for jidx in range(len(joint_vals)):
                wm.initial_joint_q[jidx] = float(joint_vals[jidx])
        self._initial_state_saved = True

    def reset_worlds(self, indices: list = None):
        """
        Reset simulation to initial state.

        Currently wraps restart_system() for full reset.
        Per-world selective reset requires future C++ support.

        Args:
            indices: Ignored (future: list of world indices to reset).
                     Currently always does full reset.
        """
        if indices is not None and len(indices) < self._world_meta.world_count:
            print("  NOTE: Per-world selective reset not yet supported; "
                  "doing full restart_system()")
        self._solver.restart_system()

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

    def get_body_pose(self, body_name, world_id: int = 0):
        """ROADMAP 1.6: Get world-frame body pose (translation + rotation quaternion)."""
        rid = self.get_body_id(body_name, world_id)
        if rid < 0:
            raise KeyError(f"Body '{body_name}' not found in world {world_id}")
        t = np.array(self._solver.get_rigid_body_translation(rid), dtype=np.float64)
        q = np.array(self._solver.get_rigid_body_rotation_quaternion(rid), dtype=np.float64)
        return t, q  # (translation_xyz, quaternion_wxyz)

    def get_body_velocity(self, body_name, world_id: int = 0):
        """ROADMAP 1.6: Get world-frame body velocity (linear + angular)."""
        rid = self.get_body_id(body_name, world_id)
        if rid < 0:
            raise KeyError(f"Body '{body_name}' not found in world {world_id}")
        v = np.array(self._solver.get_rigid_body_velocity(rid), dtype=np.float64)
        return v  # [vx, vy, vz, wx, wy, wz]

    # ───────────────────────────────────────────────────────────────

    def print_mesh_info(self):
        self._solver.print_registered_meshes_info()

    # ── Body/joint name lookup (ROADMAP B.2 + P2.1 world_id) ──────

    def get_body_id(self, name: str, world_id: int = 0) -> int:
        """Get registration ID for a body by name and optional world."""
        wm = self._world_meta
        if wm.world_count > 0:
            return wm.get_body_rid(name, world_id)
        return self._body_ids.get(name, -1)

    def get_body_name(self, rid: int) -> str:
        """Get body name from registration ID."""
        wm = self._world_meta
        if rid in wm.rid_to_body:
            bm = wm.rid_to_body[rid]
            return f"world_{bm.world_id}/{bm.name}"
        for name, body_id in self._body_ids.items():
            if body_id == rid:
                return name
        return ""

    def get_joint_index(self, body_a: str, body_b: str, world_id: int = 0) -> int:
        """Find joint index connecting two bodies by name and optional world."""
        wm = self._world_meta
        if wm.world_count > 0:
            return wm.get_joint_index(body_a, body_b, world_id)
        pair = (body_a, body_b)
        rev_pair = (body_b, body_a)
        if pair in self._joint_index_map:
            return self._joint_index_map[pair][0]
        if rev_pair in self._joint_index_map:
            return self._joint_index_map[rev_pair][0]
        return -1

    def get_joint_name(self, idx: int) -> str:
        """Get joint name by index."""
        wm = self._world_meta
        if idx in wm.jidx_to_joint:
            jm = wm.jidx_to_joint[idx]
            return f"world_{jm.world_id}/{jm.name}"
        if 0 <= idx < len(self._joint_names):
            return self._joint_names[idx]
        return ""

    def get_joint_type_by_name(self, body_a: str, body_b: str,
                               world_id: int = 0) -> str:
        """Get joint type string by body names and optional world."""
        wm = self._world_meta
        if wm.world_count > 0:
            jm = wm.get_joint_meta(body_a, body_b, world_id)
            return jm.joint_type if jm else ""
        pair = (body_a, body_b)
        rev_pair = (body_b, body_a)
        if pair in self._joint_index_map:
            return self._joint_index_map[pair][1]
        if rev_pair in self._joint_index_map:
            return self._joint_index_map[rev_pair][1]
        return ""

    # ───────────────────────────────────────────────────────────────

    def save_result(self, path):
        self._solver.save_sim_result(obj_path=path)

    def cleanup(self):
        self._solver.cleanup_device()
