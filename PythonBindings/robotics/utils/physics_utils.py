"""
Physics computation utilities for robotics simulations.
"""

import numpy as np


def estimate_rotation_matrix(ref_vertices, curr_vertices):
    c_ref = np.mean(ref_vertices, axis=0)
    c_cur = np.mean(curr_vertices, axis=0)
    x = ref_vertices - c_ref
    y = curr_vertices - c_cur
    u, _, vt = np.linalg.svd(x.T @ y)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0.0:
        vt[-1, :] *= -1.0
        r = vt.T @ u.T
    return r


def rotation_to_axis_angle(r):
    angle = np.arccos(np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0))
    if angle < 1e-8:
        return np.zeros(3), 0.0
    axis = np.array([
        r[2, 1] - r[1, 2],
        r[0, 2] - r[2, 0],
        r[1, 0] - r[0, 1],
    ])
    axis = axis / (2.0 * np.sin(angle))
    return axis, angle


def wrap_angle_rad(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def body_frame_velocity(q_base, lin_vel_world, ang_vel_world):
    q_conj = np.array([-q_base[0], -q_base[1], -q_base[2], q_base[3]])
    def quat_rotate(q, v):
        qv = np.array([*q[:3], 0.0])
        t = 2.0 * np.cross(q[:3], v)
        return v + q[3] * t + np.cross(q[:3], t)
    vel_b = quat_rotate(q_conj, lin_vel_world)
    avel_b = quat_rotate(q_conj, ang_vel_world)
    return vel_b, avel_b
