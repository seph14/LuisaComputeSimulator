"""
Rigid body mesh creation helpers for robotics simulations.
"""

import os
import numpy as np
import trimesh
import lcs_py as lcs


def project_root():
    return os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..", "..")
    )


_CUBE_MESH = None


def _load_cube_mesh():
    global _CUBE_MESH
    if _CUBE_MESH is None:
        path = os.path.join(project_root(), "Resources", "InputMesh", "cube.obj")
        _CUBE_MESH = trimesh.load(path, process=False)
    return _CUBE_MESH


def create_rigid_body(solver, name, tx=0.0, ty=0.0, tz=0.0,
                      sx=1.0, sy=1.0, sz=1.0,
                      mesh=None):
    if mesh is None:
        mesh = _load_cube_mesh()
    body = solver.create_world_data_from_array(name, mesh.vertices, mesh.faces)
    body.set_simulation_type(lcs.MaterialType.Rigid)
    body.set_translation(tx, ty, tz)
    body.set_scale_xyz(sx, sy, sz)
    return body


def create_fixed_anchor(solver, name, tx=0.0, ty=0.0, tz=0.0,
                        sx=0.02, sy=0.02, sz=0.02):
    body = create_rigid_body(solver, name, tx, ty, tz, sx, sy, sz)
    body.add_fixed_point_by_indices(np.array([0], dtype=np.int32))
    return solver.register_world_data(body)


def create_box_body(solver, name, half_extents, tx=0.0, ty=0.0, tz=0.0):
    hx, hy, hz = half_extents
    return create_rigid_body(solver, name, tx, ty, tz,
                             hx * 2.0, hy * 2.0, hz * 2.0)


def load_mesh_from_file(file_path):
    return trimesh.load(file_path, process=False)


def estimate_com(vertices):
    return np.mean(vertices, axis=0)
