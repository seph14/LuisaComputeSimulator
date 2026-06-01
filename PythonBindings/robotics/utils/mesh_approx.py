"""
Mesh approximation utilities (ROADMAP B.3).

Provides convex hull decomposition, box approximation, and mesh
simplification for collision geometry generation.
"""

import numpy as np
import trimesh


def convex_hull(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """Compute the convex hull of a mesh."""
    return mesh.convex_hull


def box_approximation(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """Approximate a mesh with an oriented bounding box."""
    box = mesh.bounding_box_oriented
    return trimesh.creation.box(extents=box.extents,
                                transform=box.primitive.transform)


def sphere_approximation(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """Approximate a mesh with a minimum enclosing sphere."""
    center = mesh.bounding_sphere.primitive.center
    radius = mesh.bounding_sphere.primitive.radius
    return trimesh.creation.icosphere(radius=radius, subdivisions=2)


def capsule_approximation(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """Approximate a mesh with a capsule along its principal axis."""
    # Use PCA to find principal axis
    verts = mesh.vertices - mesh.centroid
    cov = verts.T @ verts / len(verts)
    _, _, vh = np.linalg.svd(cov)
    principal_axis = vh[0]  # longest axis
    # Project vertices onto principal axis
    proj = verts @ principal_axis
    length = proj.max() - proj.min()
    # Radius from perpendicular spread
    perp_spread = np.linalg.norm(verts - np.outer(proj, principal_axis), axis=1).max()
    radius = perp_spread * 0.8  # slightly conservative

    # Create capsule along Z, then rotate to principal axis
    capsule = trimesh.creation.capsule(radius=float(radius),
                                       height=float(length))
    # Rotate Z-axis to principal_axis
    z_axis = np.array([0, 0, 1])
    if np.abs(np.dot(principal_axis, z_axis)) < 0.999:
        rot_axis = np.cross(z_axis, principal_axis)
        rot_axis /= np.linalg.norm(rot_axis)
        angle = np.arccos(np.dot(z_axis, principal_axis))
        rot = trimesh.transformations.rotation_matrix(angle, rot_axis)
        capsule.apply_transform(rot)
    capsule.apply_translation(mesh.centroid)
    return capsule


def simplify_mesh(mesh: trimesh.Trimesh, target_faces: int = 100) -> trimesh.Trimesh:
    """Simplify mesh to a target face count for collision geometry."""
    if len(mesh.faces) <= target_faces:
        return mesh
    ratio = target_faces / len(mesh.faces)
    return mesh.simplify_quadric_decimation(face_count=target_faces)


def generate_collision_proxies(mesh: trimesh.Trimesh,
                               methods=("convex_hull", "box", "capsule")) -> dict:
    """Generate multiple collision proxy geometries for a mesh."""
    results = {}
    if "convex_hull" in methods:
        results["convex_hull"] = convex_hull(mesh)
    if "box" in methods:
        results["box"] = box_approximation(mesh)
    if "capsule" in methods:
        results["capsule"] = capsule_approximation(mesh)
    if "sphere" in methods:
        results["sphere"] = sphere_approximation(mesh)
    return results
